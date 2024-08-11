#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <assert.h>
#include <math.h>
#include "dlimitor.h"

#define INTP_PASS 0
#define INTP_DROP 1
#define min(a, b) ((a) > (b) ? (b) : (a))
#define cas(dst, old, new) __sync_bool_compare_and_swap((dst), (old), (new))
/* weights [1-a,a] becomes [(1-a)^n, 1-(1-a)^n] if no arrivals within n steps */
#define sliding_steps(v, w, n) ((v) > 0 ? ((v) * pow(1 - 1.0/(1 << (w)), (n))) : 0)


int dlimitor_update_config(dlimitor_t *limitor, dlimitor_cfg_t *cfg)
{
    uint64_t i;
    #define GET_HZ_MAX 4500000000
    #define cfg_init(name, max)  do { if (cfg->name > 0 && cfg->name <= max && limitor->cfg.name == 0) \
                                    { limitor->cfg.name = cfg->name; }} while(0)
    #define cfg_copy(name, max)  do { if (cfg->name > 0 && cfg->name <= max && limitor->cfg.name != cfg->name) \
                                    { limitor->cfg.name = cfg->name; }} while(0)
    #define cfg_sync_numa(name, value) do { uint64_t j; for (j = 0; j < limitor->cfg.numa_num; j++) \
                                    { if (limitor->numas[j]->name != (value)) { limitor->numas[j]->name = (value); }}} while(0)
    cfg_copy(update_interval, GET_HZ_MAX); /* max update_interval: 1 second */
    cfg_sync_numa(numa_update_interval, limitor->cfg.update_interval);
    cfg_copy(second_ticks, GET_HZ_MAX);
    cfg_copy(sliding_power_w, 8);
    cfg_copy(intp_power_k, 24);
    cfg_sync_numa(mask, (1<<limitor->cfg.intp_power_k)-1);
    cfg_copy(qos_level_num, QOS_LEVEL_MAX);
    cfg_sync_numa(counter_num, limitor->cfg.qos_level_num * 2);
    cfg_init(numa_num, NUMA_MAX);
    cfg_init(worker_num_per_numa, 64);
    cfg_sync_numa(worker_num, limitor->cfg.worker_num_per_numa);
    cfg_copy(limit_total, GET_HZ_MAX);
    for (i = 0; i < QOS_LEVEL_MAX; i++)
        cfg_copy(limits[i], GET_HZ_MAX);
    return 0;
}


int dlimitor_init(dlimitor_t *limitor, numa_update_t *numas[], dlimitor_cfg_t *cfg)
{
    uint64_t size = sizeof(numa_update_t);
    int i = 0;
    memset(limitor, 0, sizeof(*limitor));
    memcpy(&limitor->cfg, cfg, sizeof(*cfg));
    size += limitor->cfg.worker_num_per_numa * COUNTER_MAX * sizeof(uint64_t);
    for (i = 0; i < limitor->cfg.numa_num; i++)
    {
        memset(numas[i], 0, size);
        numas[i]->numa_update_interval = limitor->cfg.update_interval;
        numas[i]->counter_num = limitor->cfg.qos_level_num * 2;
        numas[i]->worker_num = limitor->cfg.worker_num_per_numa;
        numas[i]->mask = (1 << limitor->cfg.intp_power_k) - 1;
        limitor->numas[i] = numas[i];
    }
    return 0;
}

int dlimitor_numa_update(numa_update_t *numa)
{
    int i, j;
    uint64_t sum[COUNTER_MAX] = {0};
    for (j = 0; j < numa->worker_num; j++)
        for (i = 0; i < numa->counter_num; i++)
            sum[i] += numa->worker[j][i];
    for (i = 0; i < numa->counter_num; i++)
        numa->sum[i] = sum[i];
    return 0;
}

int dlimitor_host_update(dlimitor_t *limitor, uint64_t duration)
{
    uint64_t i, j, k, rxps, txps, intp, fixp, limit, remaining;
    double freq, alpha, beta, new;
    uint64_t sum[COUNTER_MAX] = {0};

    if (duration < limitor->cfg.update_interval)
        return 0;
    freq = (limitor->cfg.second_ticks * 1.0 / duration);
#ifdef DLIMITOR_DEBUG
    beta = duration * 1.0 / limitor->cfg.update_interval;
    if (beta < 100000 && beta > limitor->stats.duration_intervals_max)
        limitor->stats.duration_intervals_max = beta;
    limitor->stats.duration_intervals_avg *= 0.9;
    limitor->stats.duration_intervals_avg += 0.1 * beta;
#endif
    /* beta = (1 - 1/2^w)^n, sliding = beta * sliding_old + (1 - beta) * new_value */
    beta = pow ((1.0 - 1.0 / (1 << limitor->cfg.sliding_power_w)), \
                (duration * 1.0 / limitor->cfg.update_interval));
    alpha = 1.0 - beta;
    k = (limitor->cfg.qos_level_num << 1);
    for (i = 0; i < limitor->cfg.numa_num; i++)
        for (j = 0; j < k; j++)
            sum[j] += limitor->numas[i]->sum[j];
    for (i = 0; i < k; i++)
    {
        j = limitor->sum[i];
        new = (sum[i] > j) ? (sum[i] - j) * freq : 0.0; /* new nps: Xn */
        limitor->sum[i] = sum[i]; 
        j = limitor->nps[i];  /* old move avg */
        limitor->nps[i] = j > 0 ? (j * beta + new * alpha) : new; /* new move avg */
        limitor->err[i] = beta * (limitor->err[i] + alpha * (new - j) * (new - j));
    }
    k = limitor->cfg.intp_power_k;
    remaining = limitor->cfg.limit_total;
    for (i = 0; i < limitor->cfg.qos_level_num; i++)
    {
        intp = 0;
        if (remaining > 0)
        {
            rxps = limitor->nps[2 * i];
            txps = limitor->nps[2 * i + 1];
            limit = limitor->cfg.limits[i];
            limit = min (limit, remaining);
            remaining -= min (limit, txps);
            rxps = (rxps > 0 ? rxps : 1);
            intp = (limit << k) / rxps;
            fixp = (txps << k) / rxps;
            fixp = (fixp > intp) ? (fixp - intp) << 4 : 0;  /* scale num that fixp exceeds intp */
            intp = (intp > fixp) ? (intp - fixp) : intp;    /* fix intp */
            intp = (intp > (1 << k)) ? (1 << k) : intp;     /* resize to 2^k */
        }
        /* intp = limitor->intp[i] * beta + intp * (1 - beta); */
        for (j = 0; j < limitor->cfg.numa_num; j++)
            if (limitor->numas[j]->intp[i] != intp)
                limitor->numas[j]->intp[i] = intp;
    }
#ifdef DLIMITOR_DEBUG
    j = limitor->stats.atomic_fails_last;
    k = limitor->stats.atomic_fails;
    j = k > j ? (k - j) * freq : 0;
    limitor->stats.atomic_fails_last = k;
    limitor->stats.atomic_fails_rate  = limitor->stats.atomic_fails_rate  * beta + j * (1 - beta);
#endif
    return 0;
}

int dlimitor_worker_update(dlimitor_t *limitor, int numa_id, int core_id,
                           int qos, uint64_t pkt_num, uint64_t rndint, uint64_t curr_time)
{
    int ret = INTP_DROP;
    numa_update_t *numa = limitor->numas[numa_id];
    uint64_t prev_time, next_time, *c = &numa->worker[core_id][qos<<1];

    c[0] += pkt_num;
    if ((rndint & numa->mask) < numa->intp[qos])
    {
        c[1] += pkt_num;
        ret = INTP_PASS;
    }
    if ((rndint & 0xff) > 15) // + (rate & 0xff) > 16)
        return ret;
    prev_time = numa->update_next_time;
    if (curr_time < prev_time)
        return ret;
    next_time = curr_time + numa->numa_update_interval;
    if (!cas(&numa->update_next_time, prev_time, next_time)) {
        DLIMITOR_DEBUG_INC(limitor->stats.atomic_fails);
        return ret;
    }
    dlimitor_numa_update(numa);
    prev_time = limitor->update_next_time;
    if (curr_time < prev_time)
        return ret;
    next_time = curr_time + limitor->cfg.update_interval;
    if (!cas(&limitor->update_next_time, prev_time, next_time)) {
        DLIMITOR_DEBUG_INC(limitor->stats.atomic_fails);
        return ret;
    }
    dlimitor_host_update(limitor, (next_time - prev_time));
    return ret;
}

int dlimitor_host_stats (dlimitor_t * limitor, uint64_t curr_time)
{
    static uint64_t prev_counters[COUNTER_MAX] = {0};
    static uint64_t start_time = 0, prev_time = 0;
    uint64_t curr_counters[COUNTER_MAX] = {0};
    char *b9 = "         ";
    char *b3 = "   ";
    uint64_t j, rx, tx, rxpps, txpps, rxps, txps, tx_rx, limit, intp;
    double sleep_secs, total_secs, rxerr, txerr;

    if (start_time == 0) {
        memcpy(prev_counters, limitor->sum, 64);
        prev_time = curr_time;
        start_time = curr_time;
        return 0;
    }
    memcpy(curr_counters, limitor->sum, 64);
    sleep_secs = (curr_time - prev_time) * 1.0 / limitor->cfg.second_ticks;
    total_secs = (curr_time - start_time) * 1.0 / limitor->cfg.second_ticks;
    printf ("--------------------------------------------------------------------------------------------\n");
    printf ("qos qos_limit   sliding-rxps-real-stderror sliding-txps-real-stderror int_p  tx/rx  rx_count%s tx_count%s total_rxpps%s total_txpps%s \n", 
            b9, b9, b3, b3);
    for (j = 0; j < limitor->cfg.qos_level_num; j++) {
        limit = limitor->cfg.limits[j];
        rxps = limitor->nps[2*j];
        rxerr = sqrt(limitor->err[2*j])/rxps;
        txps = limitor->nps[2*j+1];
        txerr = sqrt(limitor->err[2*j+1])/txps;
        rx = curr_counters[2*j];
        tx = curr_counters[2*j+1];
        intp = limitor->numas[0]->intp[j];
        tx_rx = (txps << limitor->cfg.intp_power_k) / (rxps>0?rxps:1); /* void divide by zero */
        rxpps = (uint64_t)((rx - prev_counters[2*j]) * 1.0 / sleep_secs);
        txpps = (uint64_t)((tx - prev_counters[2*j+1]) * 1.0 / sleep_secs);
        printf ("%-3lu %-11lu %-8lu/%-8lu %-.6f %-8lu/%-8lu %-.6f %-6lu %-6lu %-17lu %-17lu %-14lu %-14lu\n", 
                j, limit, rxps, rxpps, rxerr, txps, txpps, txerr, intp, tx_rx, rx, tx,
                (uint64_t)(rx * 1.0 / total_secs), (uint64_t)(tx * 1.0 / total_secs));
    }
    printf ("---------------------------------------------------------------------------------------------\n");
    printf ("limit_total=%lu, sliding_w=%lu, intp_k=%lu, qos_level_num=%lu\n",
           limitor->cfg.limit_total, limitor->cfg.sliding_power_w, 
	    limitor->cfg.intp_power_k, limitor->cfg.qos_level_num);
    printf("update_interval=%lu, second_ticks=%lu, numa_num=%lu, worker_num_per_numa=%lu\n",
		  limitor->cfg.update_interval, limitor->cfg.second_ticks,
		  limitor->cfg.numa_num, limitor->cfg.worker_num_per_numa);
#ifdef DLIMITOR_DEBUG
    printf("atomic fails=%lu, last=%lu, fails_rate=%.3f\n", \
        limitor->stats.atomic_fails, limitor->stats.atomic_fails_last, limitor->stats.atomic_fails_rate);
    printf("duration/interval max=%.3f, avg=%.3f\n", limitor->stats.duration_intervals_max, limitor->stats.duration_intervals_avg);
#endif
    printf ("escaped_time=%.3fs\n\n", total_secs);
    memcpy(prev_counters, curr_counters, 64);
    prev_time = curr_time;
    fflush (stdout);
    return 0;
}



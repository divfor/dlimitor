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
#define cas(dst, old, new) __sync_bool_compare_and_swap((dst), (old), (new))

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
    cfg_sync_numa(qos_num, limitor->cfg.qos_level_num);
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
        numas[i]->memsize = size;
        numas[i]->numa_update_interval = limitor->cfg.update_interval;
        numas[i]->qos_num = limitor->cfg.qos_level_num;
        numas[i]->worker_num = limitor->cfg.worker_num_per_numa;
        numas[i]->mask = (1 << limitor->cfg.intp_power_k) - 1;
        limitor->numas[i] = numas[i];
    }
    return 0;
}

int dlimitor_numa_update(numa_update_t *numa)
{
    int i, j;
    uint64_t counter_num = numa->qos_num << 1;
    uint64_t sum[COUNTER_MAX] = {0};
    for (j = 0; j < numa->worker_num; j++)
        for (i = 0; i < counter_num; i++)
            sum[i] += numa->worker[j][i];
    for (i = 0; i < counter_num; i++)
        numa->sum[i] = sum[i];
    return 0;
}

int dlimitor_host_update(dlimitor_t *limitor, uint64_t duration)
{
    uint64_t i, j, n, add, old, nps, rxps, txps, intp;
    uint64_t w = limitor->cfg.sliding_power_w;
    uint64_t k = limitor->cfg.intp_power_k;
    uint64_t counter_num = limitor->cfg.qos_level_num << 1;
    uint64_t limit, remaining;
    uint64_t sum[COUNTER_MAX] = {0};

    if ((n = round(duration / limitor->cfg.update_interval)) < 1)
        return 0;
    for (i = 0; i < limitor->cfg.numa_num; i++)
        for (j = 0; j < counter_num; j++)
            sum[j] += limitor->numas[i]->sum[j];
    for (i = 0; i < counter_num; i++)
    {
        old = limitor->sum[i];
        /* assert(sum[i] > limitor->sum[i]) */
        add = sum[i] > old ? sum[i] - old : 0;
        limitor->sum[i] = sum[i];
        nps = limitor->nps[i];
        if ((old = nps) > 0) {
            /* weight (1-a) becomes (1-a)^n if no arrivals within n steps */
            /* weighted old value: nps = nps * (1-1/2^w)^n */
            if (n < 100)
                for (j = 0; j < n; j++, nps -= (nps >> w));
            else
                nps *= pow(((1<<w)-1)*1.0/(1<<w), n);
            /* weighted new value: add = add * (1 - (1-a)^n) = add * (1 - (ew_nps/orig_nps)) */
            add -= add * nps / old;
        }
        limitor->nps[i] = nps + add * limitor->cfg.second_ticks / duration;
    }
    remaining = limitor->cfg.limit_total;
    for (i = 0; i < limitor->cfg.qos_level_num; i++)
    {
        intp = 0;
        if (remaining > 0)
        {
            limit = limitor->cfg.limits[i];
            limit = limit > remaining ? remaining : limit;
            rxps = limitor->nps[2 * i];
            txps = limitor->nps[2 * i + 1];
            n = (txps > limit) ? (txps - limit) : 0;        /* n = how many txps exceeds limit */
            n = limit > 0 ? (n << 4) * txps / limit : 0;    /* n = diff scaled up */
            n = limit > n ? limit - n : limit;              /* n = limit fixed back */
            intp = (n << k) / (rxps > 0 ? rxps : 1);        /* rxps * p = fixed_limit, intp = (p << k) */
            remaining -= (limit > txps ? txps : limit);
        }
        limitor->intp[i] = intp;
        for (j = 0; j < limitor->cfg.numa_num; j++)
            if (intp != limitor->numas[j]->intp[i])
                limitor->numas[j]->intp[i] = intp;
    }
    return 0;
}

int dlimitor_worker_update(dlimitor_t *limitor, int numa_id, int core_id,
                           int pkt_qos_level, uint64_t pkt_num, uint64_t rndint, uint64_t curr_time)
{
    uint64_t prev_time, next_time;
    int ret = INTP_DROP;
    numa_update_t *numa = limitor->numas[numa_id];
    rxtx_t *level = (rxtx_t *)&(numa->worker[core_id][2 * pkt_qos_level]);
    level->rx += pkt_num;
    if ((rndint & numa->mask) < numa->intp[pkt_qos_level])
    {
        level->tx += pkt_num;
        ret = INTP_PASS;
    }
    prev_time = numa->update_next_time;
    if (curr_time < prev_time || (rndint & 0xf) < 15)
        return ret;
    next_time = curr_time + numa->numa_update_interval + (rndint & 0xf);
    if (!cas(&numa->update_next_time, prev_time, next_time)) {
        limitor->numa_atomic_fails++;
        return ret;
    }
    dlimitor_numa_update(numa);
    prev_time = limitor->update_next_time;
    if (curr_time < prev_time)
        return ret;
    next_time = curr_time + limitor->cfg.update_interval;
    if (!cas(&limitor->update_next_time, prev_time, next_time)) {
        limitor->host_atomic_fails++;
        return ret;
    }
    dlimitor_host_update(limitor, (next_time - prev_time));
    return ret;
}

int dlimitor_host_stats (dlimitor_t * limitor, uint64_t escaped)
{
    char *b10 = "          ";
    int j;
    uint64_t rx, tx, rxps, txps, tx_rx, limit, intp;
    printf ("-----------------------------------------------------------------------------------------\n");
    printf ("qos qos_limit   rxps%s txps%s int_p  tx/rx  rx_count%s   tx_count%s   rxpps%stxpps%s\n", 
             b10, b10, b10, b10, b10, b10);
    for (j = 0; j < limitor->cfg.qos_level_num; j++) {
       limit = limitor->cfg.limits[j];
       rxps = limitor->nps[2*j];
       txps = limitor->nps[2*j+1];
       rx = limitor->sum[2*j];
       tx = limitor->sum[2*j+1];
       intp = limitor->intp[j];
       tx_rx = (txps << limitor->cfg.intp_power_k) / (rxps>0?rxps:1); /* void divide by zero */
        printf ("%-3d %-11lu %-14lu %-14lu %-6lu %-6lu %-20lu %-20lu %-14lu %-14lu\n", 
                  j,  limit, rxps, txps, intp, tx_rx, rx, tx, 
		  rx * limitor->cfg.second_ticks / escaped, tx * limitor->cfg.second_ticks / escaped);
    }
  printf ("------------------------------------------------------------------------------------------\n");
  printf ("limit_total=%lu, sliding_w=%lu, intp_k=%lu, qos_num=%lu\n",
          limitor->cfg.limit_total, limitor->cfg.sliding_power_w, 
	  limitor->cfg.intp_power_k, limitor->cfg.qos_level_num);
  printf("update_interval=%lu, second_ticks=%lu, numa_num=%lu, worker_num_per_numa=%lu\n",
		  limitor->cfg.update_interval, limitor->cfg.second_ticks,
		  limitor->cfg.numa_num, limitor->cfg.worker_num_per_numa);
  printf("atomic fails: numa=%lu, host=%lu\n", limitor->numa_atomic_fails, limitor->host_atomic_fails);
  if (escaped > 0)
     printf ("escaped_time=%.3fs\n\n", escaped * 1.0 / limitor->cfg.second_ticks);
  fflush (stdout);
  return 0;
}


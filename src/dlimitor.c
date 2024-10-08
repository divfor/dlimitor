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
//* weights [1-a,a] becomes [(1-a)^n, 1-(1-a)^n] if no arrivals within n steps */
#define sliding_steps(v, w, n) ((v) > 0 ? ((v) * pow(1 - 1.0/(1 << (w)), (n))) : 0)


int dlimitor_host_stats (numa_sum_t *numa, uint64_t curr_time)
{
    static uint64_t prev_counters[COUNTER_MAX] = {0};
    static uint64_t start_time = 0, prev_time = 0;
    uint64_t curr_counters[COUNTER_MAX] = {0};
    uint64_t j, rx, tx, rxpps, txpps, rxps, txps, tx_rx, limit, intp;
    double sleep_secs, total_secs, rxerr, txerr;
    host_sum_t *limitor = numa->limitor;
    dlimitor_cfg_t *cfg = limitor->cfg;
    dlimitor_qos_t *qos = (limitor->qos.qos_num > 0) ? &limitor->qos : (dlimitor_qos_t *)(limitor->cfg + 1);

    if (start_time == 0) {
        memcpy(prev_counters, limitor->sum, 64);
        prev_time = curr_time;
        start_time = curr_time;
        return 0;
    }
    memcpy(curr_counters, limitor->sum, 64);
    sleep_secs = (curr_time - prev_time) * 1.0 / cfg->second_ticks;
    total_secs = (curr_time - start_time) * 1.0 / cfg->second_ticks;
    printf ("--------------------------------------------------------------------------------------------\n");
    printf ("qos qos_limit   sliding-rxps-real-stderror     sliding-txps-real-stderror int_p  tx/rx  total_rxpps    total_txpps\n");
    for (j = 0; j < qos->qos_num; j++) {
        limit = qos->limits[j];
        rxps = limitor->nps[2*j];
        rxerr = sqrt(limitor->var[2*j])/(rxps>0?rxps:1);
        txps = limitor->nps[2*j+1];
        txerr = sqrt(limitor->var[2*j+1])/(txps>0?txps:1);
        rx = curr_counters[2*j];
        tx = curr_counters[2*j+1];
        intp = numa->intp[j];
        tx_rx = (txps << INTP_PWR_K) / (rxps>0?rxps:1); /* void divide by zero */
        rxpps = (uint64_t)((rx - prev_counters[2*j]) * 1.0 / sleep_secs);
        txpps = (uint64_t)((tx - prev_counters[2*j+1]) * 1.0 / sleep_secs);
        printf ("%-3lu %-11lu %-10lu/%-10lu %-.6f %-8lu/%-8lu %-.6f %-6lu %-6lu %-14lu %-14lu\n",
                j, limit, rxps, rxpps, rxerr, txps, txpps, txerr, intp, tx_rx,
                (uint64_t)(rx * 1.0 / total_secs), (uint64_t)(tx * 1.0 / total_secs));
    }
    printf ("---------------------------------------------------------------------------------------------\n");
    printf ("limit_total=%d, sliding_w=%d, intp_k=%d, qos_num=%d\n",
           qos->limit_total, cfg->sliding_power_w, INTP_PWR_K, qos->qos_num);
    printf("update_interval=%d, second_ticks=%d, numa_num=%d, worker_num=%d, ugly_intp=%d, flags=%x\n",
		  cfg->update_interval, cfg->second_ticks, NUMA_MAX, numa->worker_num, numa->ugly_intp, cfg->flags);

    printf ("escaped_time=%.3fs\n", total_secs);
    memcpy(prev_counters, curr_counters, 64);
    prev_time = curr_time;
    fflush (stdout);
    return 0;
}


int dlimitor_host_update(host_sum_t *limitor, uint64_t duration)
{
    uint64_t i, j, k, rxps, txps, intp, fixp, limit, remaining;
    double freq, alpha, beta, new;
    uint64_t sum[COUNTER_MAX] = {0};
    const dlimitor_cfg_t * const cfg = limitor->cfg;
    const dlimitor_qos_t * const qos = (limitor->qos.qos_num > 0) ? &limitor->qos : (dlimitor_qos_t *)(limitor->cfg + 1);

    if (duration < cfg->update_interval)
        return 0;
    freq = (cfg->second_ticks * 1.0 / duration);
    /* beta = (1 - 1/2^w)^n, sliding = beta * sliding_old + (1 - beta) * new_value */
    beta = pow ((1.0 - 1.0 / (1 << cfg->sliding_power_w)), (duration * 1.0 / cfg->update_interval));
    alpha = 1.0 - beta;
    k = (qos->qos_num << 1);
    for (i = 0; i < NUMA_MAX; i++)
        for (j = 0; j < k; j++)
            sum[j] += limitor->numas[i]->sum[j];
    for (i = 0; i < k; i++) {
        j = limitor->sum[i];
        new = (sum[i] > j) ? (sum[i] - j) * freq : 0.0; /* new nps: Xn */
        limitor->sum[i] = sum[i];
        j = limitor->nps[i];  /* old move avg */
        limitor->nps[i] = j > 0 ? (j * beta + new * alpha) : new; /* new move avg */
        limitor->var[i] = beta * (limitor->var[i] + alpha * (new - j) * (new - j));
    }
    /* k = limitor->cfg.intp_power_k; */
    remaining = qos->limit_total;
    for (i = 0; i < qos->qos_num; i++) {
        intp = 0;
        if (remaining > 0)
        {
            rxps = limitor->nps[(i << 1)];
            txps = limitor->nps[(i << 1) + 1];
            limit = qos->limits[i];
            limit = min (limit, remaining);
            remaining -= min (limit, txps);
            rxps = (rxps > 0 ? rxps : 1);
            intp = (limit << INTP_PWR_K) / rxps;
            fixp = (txps << INTP_PWR_K) / rxps;
            fixp = (fixp > intp) ? (fixp - intp) : 0;       /* scale num that fixp exceeds intp */
            fixp = fixp>4 ? fixp<<5 : fixp<<fixp;
            intp = (intp > fixp) ? (intp - fixp) : intp;    /* fix intp */
            intp = (intp > INTP_MAX) ? INTP_MAX : intp;     /* resize to 2^k */
        }
        /* intp = limitor->intp[i] * beta + intp * (1 - beta); */
        for (j = 0; j < NUMA_MAX; j++)
            if (limitor->numas[j]->intp[i] != intp)
                limitor->numas[j]->intp[i] = intp;
    }
    return 0;
}

int dlimitor_numa_update(numa_sum_t *numa)
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

int dlimitor_worker_update(numa_sum_t *numa, int numa_nth_core, int qos, int pkt_num, uint32_t rndint, uint64_t curr_time)
{
    int ret = INTP_DROP;
   
    while (numa_nth_core >= numa->worker_num)
        numa_nth_core -= numa->worker_num;
    uint64_t * const c = &numa->worker[numa_nth_core][qos<<1];
    c[0] += pkt_num;
    if ((rndint & INTP_MASK) < numa->intp[qos]) {
        c[1] += pkt_num;
        ret = INTP_PASS;
    }
    if ((rndint & 0xff) > 15)
        return ret;
    uint64_t prev_time = numa->update_next_time;
    if (curr_time < prev_time)
        return ret;
    uint64_t next_time = curr_time + numa->numa_update_interval;
    if (!cas(&numa->update_next_time, prev_time, next_time))
        return ret;
    dlimitor_numa_update(numa);
    if (!numa->limitor)
       return ret;
    prev_time = numa->limitor->update_next_time;
    if (curr_time < prev_time)
        return ret;
    host_sum_t *limitor = numa->limitor;
    next_time = curr_time + limitor->cfg->update_interval;
    if (!cas(&limitor->update_next_time, prev_time, next_time))
        return ret;
    dlimitor_host_update(limitor, (next_time - prev_time));
    return ret;
}

int dlarray_free (dlarray_t *da) {
    int sk, hsz;
    da_skmem_t *skm;

    for (sk = 0; sk < NUMA_MAX; sk++) {
        skm = da->skmem[sk];
        if (da->cfg.flags & PTR_BUCKET) {
            int i;
            for (i = 0; i <= skm->nb_szmask; i++)
                mem_free(skm->numa_buckets.p[i], skm->nb_membytes);
            hsz = (skm->nb_szmask+1) >> 1;
            hsz = hsz > 0 ? hsz : 1;
            for (i = 0; i < hsz; i++)
                mem_free (skm->host_buckets.p[i], sizeof(host_sum_t));
        }
        if (skm)
            mem_free(da->skmem[sk], da->skmem[sk]->skm_membytes);
        if (da->bmf[sk])
            mem_free(da->bmf[sk], sizeof(bmfilter_t));
    }
    return 0;
}

int dlarray_init (dlarray_t *da, uint32_t dlimitor_max_num, uint32_t total_worker_num,
                  dlimitor_cfg_t *cfg, dlimitor_qos_t *default_qos) {
    da_skmem_t *skm;
    uint32_t sk, num, hnum, numa_worker_num = (total_worker_num >> NUMA_PWR);
    uint32_t membytes_skm, membytes_host_buckets, membytes_numa_buckets;
    uint32_t host_bucket_size = sizeof(host_sum_t);
    uint32_t numa_bucket_size = sizeof(numa_sum_t) + numa_worker_num * COUNTER_MAX * sizeof(uint64_t);


    memset(da, 0, sizeof(*da));
    da->cfg.second_ticks = 1000000;
    da->cfg.sliding_power_w = 4;
    da->cfg.update_interval = da->cfg.second_ticks / ((2 << da->cfg.sliding_power_w) - 1);
    da->cfg.flags = DLIMITOR_CFG_FLAGS;
    if(cfg)
        memcpy(&da->cfg, cfg, sizeof(*cfg));

    for (num = 1; num < dlimitor_max_num; num = num << 1);
    hnum = (num >> NUMA_PWR) > 0 ? (num >> NUMA_PWR) : 1; /* fix one limitor */
    printf("dlimitor max number is set to [%d]\n", num);

    if (da->cfg.flags & PTR_BUCKET) {
        membytes_host_buckets = hnum * sizeof(host_sum_t *);
        membytes_numa_buckets = num * sizeof(numa_sum_t *);
    } else {
        membytes_host_buckets = hnum * host_bucket_size;
        membytes_numa_buckets = num * numa_bucket_size;
    }
    membytes_skm = sizeof(*skm) + membytes_numa_buckets + membytes_host_buckets;
    
    for (sk = 0; sk < NUMA_MAX; sk++) {
        printf("Allocate memory for dlimitor array: buckets=[%d] sk=[%d] ... ", num, sk);
        if (!(skm = (da_skmem_t *) mem_alloc (membytes_skm, sk, "dlimitor_skmem")))
            goto ERR_RETURN;
        memset(skm, 0, membytes_skm);
        printf("done\n");
        memcpy(&skm->cfg, &da->cfg, sizeof(da->cfg));
        if(default_qos)
             memcpy(&skm->qos, default_qos, sizeof(*default_qos));
        skm->worker_num = numa_worker_num;
        skm->nb_szmask = num - 1;
        skm->nb_membytes = numa_bucket_size;
        skm->skm_membytes = membytes_skm;
        if (da->cfg.flags & PTR_BUCKET) {
            skm->host_buckets.p = (host_sum_t **)((uint64_t)skm + sizeof(*skm) + membytes_numa_buckets);
        } else {
            skm->host_buckets.a = (host_sum_t *)((uint64_t)skm + sizeof(*skm) + membytes_numa_buckets);
        }
        da->skmem[sk] = skm;
    }
    return num;
ERR_RETURN:
    dlarray_free(da);
    return -1;
}

int dlarray_cfg_update (dlimitor_cfg_t *cfg, dlarray_t *da){
    int sk;
    if (da) {
        memcpy(&da->cfg, cfg, sizeof(*cfg));
        for (sk = 0; sk < NUMA_MAX; sk++)
            memcpy(&da->skmem[sk]->cfg, cfg, sizeof(*cfg));
        return 0;
    }
    return -1;
}

int dlarray_qos_update (dlimitor_qos_t *qos, dlarray_t *da, dlimitor_t *limitor){
    int sk;
    if (limitor)
        memcpy(&(limitor->qos), qos, sizeof(*qos));
    if (da) {
        for (sk = 0; sk < NUMA_MAX; sk++)
            memcpy(&(da->skmem[sk]->qos), qos, sizeof(*qos));
    }
    return 0;
}

int dlarray_add_limitor (dlarray_t *da, uint32_t key, dlimitor_qos_t *custom_qos) {
    uint32_t hb_sk, hb_idx, nb_idx, sk;
    dlimitor_qos_t *qos;
    numa_sum_t *numa, *dlnuma[NUMA_MAX] = {0};
    host_sum_t *dlhost = NULL;
    da_skmem_t *skm = da->skmem[0];
    uint32_t numa_sum_size = sizeof(numa_sum_t) + skm->worker_num * COUNTER_MAX * sizeof(uint64_t);

    nb_idx = key & skm->nb_szmask;
    hb_sk = nb_idx & NUMA_MASK;
    hb_idx = nb_idx >> NUMA_PWR;

    /* allocate memory and initialize 1 x host_sum_t */
    if (da->cfg.flags & PTR_BUCKET) {
        uint32_t size = sizeof(*dlhost);
        if (!(dlhost = (host_sum_t *) mem_alloc (size, hb_sk, "dlimitor-dlhost"))) {
            printf("rte_malloc_socket dlimitor-dlhost fails\n");
            goto ERROR_RETURN;
        }
        memset(dlhost, 0, size);
        dlhost->key = key;
        if (!cas(&(da->skmem[hb_sk]->host_buckets.p[hb_idx]), NULL, dlhost)) {
            da->dlarray_add_fails++;
            printf("CAS fails: cas(&(da->skmem[%d]->hb[%d]), NULL, dlhost\n", hb_sk, hb_idx);
            goto ERROR_RETURN;
        }
    } else {
        dlhost = (host_sum_t *)(&(da->skmem[hb_sk]->host_buckets.a[hb_idx]));
        if (!cas(&dlhost->key, 0U, key)) {
            da->dlarray_add_fails++;
            printf("CAS fails: cas(&(da->skmem[%d]->hb[%d].key), NULL, [%d]\n", hb_sk, hb_idx, key);
            dlhost = NULL;
            goto ERROR_RETURN;
        }
    }
    if (custom_qos)
        memcpy(&dlhost->qos, custom_qos, sizeof(*custom_qos));
    dlhost->hb_sk = hb_sk;
    dlhost->cfg = &(da->skmem[hb_sk]->cfg);

    /* allocate memory and initialize 2 x numa_sum_t */
    for (sk = 0; sk < NUMA_MAX; sk++) {
        skm = da->skmem[sk];
        qos = (dlhost->qos.qos_num > 0) ? &dlhost->qos : &skm->qos;
        if (skm->cfg.flags & PTR_BUCKET) {
            assert(numa_sum_size == skm->nb_membytes);
            if (!(numa = (numa_sum_t *) mem_alloc(numa_sum_size, sk, "dlimitor-dlnuma"))) {
                printf("rte_malloc_socket dlimitor-dlnuma, sk=[%d] fails\n", sk);
                goto ERROR_RETURN;
            }
            memset(numa, 0, numa_sum_size);
            skm->numa_buckets.p[nb_idx] = numa;
            dlnuma[sk] = numa;
        } else {
            numa = (numa_sum_t *)((char *)skm->numa_buckets.a + nb_idx * numa_sum_size);
        }
        numa->limitor = dlhost;
        if ((skm->cfg.flags | LOCAL_UPDATE) && (sk != hb_sk))
            numa->limitor = NULL;
        numa->numa_update_interval = skm->cfg.update_interval;
        numa->ugly_intp = INTP_UGLY;
        numa->worker_num = skm->worker_num;
        numa->counter_num = qos->qos_num * 2;
        dlhost->numas[sk] = numa;   
    }
    return ++da->dlimitor_count;

ERROR_RETURN:
    if (dlhost)
        mem_free(dlhost, sizeof(*dlhost));
    for (sk = 0; sk < NUMA_MAX; sk++) {
        if (dlnuma[sk])
            mem_free(dlnuma[sk], numa_sum_size);
    }
    return -1;
}


int dlimitor_init_bitmap_pwm (bmfilter_t **pbmf, uint32_t sk) {
    uint64_t i, *mem;
    bmfilter_t *bmf;
    /* allocate per numa */
    printf("numa=[%d]: bitmap_filters memory allocaiton ... ", sk);
    if(!(bmf = (bmfilter_t *)mem_alloc(sizeof(*bmf), sk, "dlimitors_bitmap_filters")))
        return -1;
    memset(bmf, 0, sizeof(*bmf));
    printf("done: [%ld] bytes\n", sizeof(*bmf));
    /* random set bit to 1 */
    printf("Initializing bits for bitmap_filters[%d]->b0\n", sk);
    mem = bmf->b0;
    for(i = 0; i <= UINT64_8MB_MASK; i++ )
        mem[i] = ~0UL;
    //    if ((random() & 0xf) < 8)
    //mem[i] = ((random()*1UL) << 32) + random();
    printf("Initializing bits for bitmap_filters[%d]->b1\n", sk);
    mem = bmf->b1;
    for(i = 0; i <= UINT64_8MB_MASK; i++ )
        if ((random() & 0xf) < 8)
            mem[i] = ((random()*1UL) << 32) + random();
    printf("Initializing bits for bitmap_filters[%d]->b2\n", sk);
    mem = bmf->b2;
    for(i = 0; i <= UINT64_4GB_MASK; i++ )
        if ((random() & 0xf) < 8)
            mem[i] = 1;	
    printf("bitmap_filters init done\n");
    *pbmf = bmf;
    return 0;
}

int dlimitor_enable_bitmap_pwm (uint64_t bmf[], uint32_t key, numa_sum_t *numa, uint32_t qos) {
    static uint64_t cnt = 0;
    if ((++cnt & 0x3) != 0)
        return 0;

    key &= 0xffffffc0;
    const uint32_t slot = (key >> 6) & UGLY_MASK;
    uint32_t cur = numa->uglyip[slot];
    
    if (cur != 0 && key != cur) {
        cas(&(numa->uglyip[slot]), cur, 0);
        SETBIT(bmf, cur);
    }
    if (cur == 0 && (numa->intp[qos] < numa->ugly_intp))
        if (cas(&(numa->uglyip[slot]), 0, key))
            CLRBIT(bmf, key);
    return 0;
}

int dlimitor_disable_bitmap_pwm (uint64_t bmf[], uint32_t key, numa_sum_t *numa) {
    if ((numa->intp[QOS_INDEX_MAX-1] >= INTP_MAX || 
         numa->intp[QOS_INDEX_MAX-2] >= INTP_MAX) &&
         numa->intp[QOS_INDEX_MAX] > numa->ugly_intp)
        SETBIT(bmf, key);
    return 0;
}
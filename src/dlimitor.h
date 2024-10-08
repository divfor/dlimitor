#ifndef __DLIMITOR__
#define __DLIMITOR__
#include <stdint.h>

#define NUMA_PWR 1
#define NUMA_MAX (1U<<NUMA_PWR)
#define NUMA_MASK (NUMA_MAX-1)
#define INTP_PWR_K 16
#define INTP_MAX (1U<<INTP_PWR_K)
#define INTP_MASK (INTP_MAX-1)
#define INTP_UGLY ((1<<(INTP_PWR_K-1)) + (1<<(INTP_PWR_K-2)))
#define UGLY_MASK 3
#define QOS_LEVEL_MAX 4
#define QOS_INDEX_MAX (QOS_LEVEL_MAX-1)
#define COUNTER_MAX (QOS_LEVEL_MAX*2)
#define LOCAL_UPDATE 0x1
#define PTR_BUCKET   0x2
#define DLIMITOR_CFG_FLAGS LOCAL_UPDATE
//#define CHKBIT(a, ip) ((a[ip>>12] >> ((ip>>6) & 0x3f)) & 0x1)
#define SETBIT(a, ip) do { a[ip>>12] |= (1UL << ((ip>>6) & 0x3f)); } while (0)
#define CLRBIT(a, ip) do { a[ip>>12] &= ~(1UL << ((ip>>6) & 0x3f)); } while (0)
#define CHKBIT(a, ip) (a[ip>>12] & (1UL << ((ip>>6) & 0x3f)))
#define cas(dst, old, new) __sync_bool_compare_and_swap((dst), (old), (new))

#define DLIMITOR_DPDK
#ifdef DLIMITOR_DPDK
#include "rte.h"
#define mem_free(p, size) do { void *t = p; if (t) { p = NULL; rte_free(t); }} while(0)
#define mem_alloc(size, sk, name) rte_zmalloc_socket(name, size, 64, sk)
#else
/* https://man7.org/linux/man-pages/man3/numa.3.html */
#include <numa.h>
#define mem_free(p, size) do { void *t = p; if (t) { p = NULL; numa_free(t, size); }} while(0)
#define mem_alloc(size, sk, name) numa_alloc_onnode (size, sk)
#endif

#define BIT_8MB_PWR      26
#define UINT64_8MB_PWR   (BIT_8MB_PWR-6)
#define UINT64_8MB_MASK  ((1UL<<UINT64_8MB_PWR)-1)
#define BIT_4GB_PWR      35
#define UINT64_4GB_PWR   (BIT_4GB_PWR-6)
#define UINT64_4GB_MASK  ((1UL<<UINT64_4GB_PWR)-1)

typedef struct numa_sum numa_sum_t;
typedef struct host_sum host_sum_t;
typedef struct host_sum dlimitor_t;

typedef struct bmfilter {
    uint64_t b0[1U << (BIT_8MB_PWR - 6)];    /* 8MB */
    uint64_t b1[1U << (BIT_8MB_PWR - 6)];    /* 8MB */
    uint64_t b2[1U << (BIT_4GB_PWR - 6)];    /* 4GB */
} bmfilter_t;

typedef struct dlimitor_cfg {
    uint32_t sliding_power_w;            /* input: sliding factor w, n = n - (n>>w) + (new>>w), suggest w=4 */
    uint32_t second_ticks;               /* how many ticks per second, 1000000 if use usecond */
    uint32_t update_interval;            /* input: duration to sum all counters, suggest second_ticks/31=32258 */
    uint32_t flags;                      /* input: flags to switch features */
} dlimitor_cfg_t;

typedef struct dlimitor_qos {
    uint32_t qos_num;                    /* input: number of qos levels (1-4) in use. counter_num = 2 x qos_num */
    uint32_t limit_total;                /* input: total limit */
    uint32_t limits[QOS_LEVEL_MAX];      /* input: number per second limits, n = 0, 1, ..., up to 3 */     
} dlimitor_qos_t;

typedef struct host_sum {                /* 3 cachelines */
    uint64_t sum[COUNTER_MAX];           /* summize calc: store sums of numa->sum[] counters to diff at next time */
    uint32_t nps[COUNTER_MAX];           /* sliding calc: diff num per second, sliding rate = rate + (new - rate)/2^w; */
    uint32_t var[COUNTER_MAX];           /* sliding calc: variance for each rxps (nps[2*j]) */
    uint64_t update_next_time;           /* volatile atomic updated by any worker of any numa */
    dlimitor_qos_t qos;                  /* local qos cfg for this dlimitor, qos.qos_num==0 means use global qos cfg */
    dlimitor_cfg_t *cfg;                 /* pointer to global config, any update to take effective immediately */
    uint32_t key;                        /* used in hash search (mod table index) and compare (confirm bucket owner) */
    uint32_t hb_sk;                      /* id of socket/NUMA where host_sum_t's memory was allocated */
    numa_sum_t *numas[NUMA_MAX];         /* pointers to numa_sum_t on each NUMA */
} host_sum_t;

typedef struct numa_sum {                /* (2 + worker_num) cachelines */
    uint32_t intp[QOS_LEVEL_MAX];        /* timely updated: global probilities of not-drop, per QoS level */
    uint64_t update_next_time;           /* volatile atomic updated by any worker of same numa */
    uint32_t numa_update_interval;       /* duration in usec to sum(counters_of_workers_of_this_numa) */
    uint32_t ugly_intp;                  /* intp to start limiting ugly */
    uint32_t counter_num;                /* number of counters in use, equal to 2 x qos_num */
    uint32_t worker_num;                 /* number of workers per numa */
    dlimitor_t *limitor;                 /* pointer to host_sum_t */
    uint32_t uglyip[UGLY_MASK + 1];      /* uint32_t padding[4] */
    uint64_t sum[COUNTER_MAX];           /* timely updated: numa-level sums, per counter type */
    uint64_t worker[0][COUNTER_MAX];     /* wait worker_num to input */
} numa_sum_t;

typedef struct da_skmem_t {              /* (1 + N) cachelines */
    dlimitor_cfg_t cfg;                  /* input: copy dlimitor_cfg_t */
    dlimitor_qos_t qos;                  /* input: default qos cfg when dlimtor.cfg.qos_num == 0 */
    uint32_t worker_num;                 /* core or thread number running on one NUMA */
    uint32_t nb_szmask;                  /* numa_sum_t (hash_table_size - 1) for quick hash index */
    uint32_t nb_membytes;                /* sizeof(num_sum_t) + worker_num x sizeof(cacheline) */
    uint32_t skm_membytes;
    union { host_sum_t **p; host_sum_t *a; } host_buckets;  /* mem of host_buckets is right after numa_buckets */
    union { numa_sum_t *p[0]; numa_sum_t a[0]; } numa_buckets; /* mem of numa_bucket is right after da_skmem_t */
} da_skmem_t;

typedef struct dlimitor_arrary {         /* 1 cacheline */
    da_skmem_t *skmem[NUMA_MAX];         /* main memory per numa */
    dlimitor_cfg_t cfg;                  /* updatable */
    uint32_t dlarray_add_fails;          /* number of fails to create/allocate dilimitors */
    uint32_t dlimitor_count;             /* number of success to create dlimitors */
    uint32_t cacheline_align_padding[2];
    bmfilter_t *bmf[NUMA_MAX];
} dlarray_t;



int dlarray_init (dlarray_t *da, uint32_t dlimitor_num, uint32_t total_workers, dlimitor_cfg_t *cfg, dlimitor_qos_t *default_qos);
int dlarray_free (dlarray_t *da);
int dlarray_add_limitor (dlarray_t *da, uint32_t key, dlimitor_qos_t *custom_qos);
// int dlarray_del_limitor (dlarray_t *da, uint32_t key);
int dlarray_cfg_update (dlimitor_cfg_t *cfg, dlarray_t *da);
int dlarray_qos_update (dlimitor_qos_t *qos, dlarray_t *da, dlimitor_t *limitor);
int dlimitor_worker_update (numa_sum_t *numa, int numa_nth_core, int pkt_qos, int pkt_num, uint32_t rndint, uint64_t curr_time);
int dlimitor_host_stats (numa_sum_t *numa, uint64_t curr_time);
int dlimitor_init_bitmap_pwm (bmfilter_t **pbmf, uint32_t sk);
int dlimitor_enable_bitmap_pwm (uint64_t bmf[], uint32_t key, numa_sum_t *numa, uint32_t qos);
int dlimitor_disable_bitmap_pwm (uint64_t bmf[], uint32_t key, numa_sum_t *numa);

#endif

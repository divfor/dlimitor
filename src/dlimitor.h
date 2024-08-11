#ifndef __DLIMITOR__
#define __DLIMITOR__
#include <stdint.h>

#define DLIMITOR_DEBUG

#define NUMA_MAX 2
#define QOS_LEVEL_MAX 4
#define COUNTER_MAX 2*QOS_LEVEL_MAX

#ifdef DLIMITOR_DEBUG
#define DLIMITOR_DEBUG_INC(a) do {a++;} while (0)
#else
#define DLIMITOR_DEBUG_INC(a) do {} while (0)
#endif

typedef struct dlimitor_stats {
    uint64_t atomic_fails;
    uint64_t atomic_fails_last;
    double   atomic_fails_rate;
    double   duration_intervals_max;
    double   duration_intervals_avg;
} dlimitor_stats_t;

typedef struct numa_update {
    uint64_t update_next_time;           /* volatile atomic updated by any worker of same numa */
    uint64_t numa_update_interval;       /* duration in usec to sum(counters_of_workers_of_this_numa) */
    uint64_t mask;                       /* mask for hash_value & (2^k - 1) */
    uint32_t counter_num;                /* number of counters in use, equal to 2 x qos_num */
    uint32_t worker_num;                 /* number of workers per numa */
    uint64_t intp[QOS_LEVEL_MAX];        /* timely updated: global probilities of not-drop, per QoS level */
    uint64_t sum[COUNTER_MAX];           /* timely updated: numa-level sums, per counter type */
    uint64_t worker[0][COUNTER_MAX];     /* wait worker_num to input */
} numa_update_t;

typedef struct dlimitor_cfg {
    uint64_t update_interval;            /* input: duration to sum all counters, suggest second_ticks/31=32258 */
    uint64_t second_ticks;               /* how many ticks per second, 1000000 if use usecond */
    uint64_t sliding_power_w;            /* input: sliding factor w, n = n - (n>>w) + (new>>w), suggest w=4 */
    uint64_t intp_power_k;               /* input: scale float p to intp to use rand() % 2^k < intp, better k=16 */
    uint64_t qos_level_num;              /* input: number of qos levels (1-4) in use. counter_num = 2 x qos_level_num */
    uint64_t numa_num;                   /* numa_num = numa_num_configured_nodes() */
    uint64_t worker_num_per_numa;        /* input: numbre of workers per numa */
    uint64_t limit_total;                /* input: total limit */
    uint64_t limits[QOS_LEVEL_MAX];      /* input: number per second limits, n = 0, 1, ..., up to 3 */
} dlimitor_cfg_t;

/* distributed limitor */
typedef struct dlimitor {
    uint64_t sum[COUNTER_MAX];           /* summize calc: store sums of numa->sum[] counters to diff at next time */
    uint64_t nps[COUNTER_MAX];           /* sliding calc: diff num per second, sliding rate = rate + (new - rate)/2^w; */
    uint64_t err[COUNTER_MAX];           /* sliding calc: variance for each rxps (nps[2*j]) */
    uint64_t update_next_time;           /* volatile atomic updated by any worker of any numa */
    dlimitor_cfg_t cfg;                  /* input: copy to init dlimitor_cfg */
    dlimitor_stats_t stats;              /* 5 uint64_t */
    numa_update_t *numas[NUMA_MAX];      /* malloc: must local to each numa node */
} dlimitor_t;

int dlimitor_init (dlimitor_t *limitor, numa_update_t *numas[], dlimitor_cfg_t *cfg);
int dlimitor_worker_update (dlimitor_t *limitor, int numa_id, int core_id, int pkt_qos, uint64_t pkt_num, uint64_t rndint, uint64_t curr_time);
int dlimitor_update_config (dlimitor_t *limitor, dlimitor_cfg_t *cfg);
int dlimitor_host_stats (dlimitor_t * limitor, uint64_t curr_time);
#endif

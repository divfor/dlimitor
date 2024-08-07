#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <sys/inotify.h>
#include <assert.h>

#ifdef NUMA_ALLOC
#include <numa.h>
#endif

#include "mtrand.h"
#include "dlimitor.h"


static char* getconfig(const char* file, const char* name)
{
/* get one field 'name' from file
pointer meaning:

...port...=...8000...
   |  |   |   |  |
  *fs |   |   |  *be    f->forward  b-> back
      *fe |   *bs       s->start    e-> end
          *equal
*/
    static char info[64];
    char *find=NULL;
    char tmp[256],fore[64],back[64];
    char *fs,*fe,*equal,*bs,*be,*start;
    FILE *fp=fopen(file, "r");
    if (!fp) return NULL;
    while(fgets(tmp,255,fp)!=NULL) {
        start=tmp;
        equal=strchr(tmp,'=');
        if(!equal) continue;
        while(isblank(*start)) ++start;
        if(*start == '#') continue;
        if(!isalpha(*start)) continue;
        fs=start;
        while(isalnum(*start)||(*start=='_')||(*start=='[')||(*start==']')||(*start=='.')) ++start;
        fe=start-1;
        strncpy(fore,fs,fe-fs+1);
        fore[fe-fs+1]='\0';
        if(strcmp(fore,name)!=0) continue;
        start=equal+1;
        while(isblank(*start)) ++start;
        bs=start;
        while(!isblank(*start)&&*start!='\n') ++start;
        be=start-1;
        strncpy(back,bs,be-bs+1);
        back[be-bs+1]='\0';
        strcpy(info,back);
	find = info;
        //fprintf(stdout, "%s --- %s\n", name, find);
        break;
    }
    fclose(fp);
    return find;
}

int read_dlimitor_config(const char *file, void *cfg)
{
    char *s;
    dlimitor_cfg_t *c = (dlimitor_cfg_t *)cfg;
    memset(c, 0, sizeof(dlimitor_cfg_t));

    if ((s = getconfig(file, "update_interval")) != NULL)
         c->update_interval = atoll(s);
    if ((s = getconfig(file, "second_ticks")) != NULL)
         c->second_ticks = atoll(s);
    if ((s = getconfig(file, "sliding_power_w")) != NULL)
         c->sliding_power_w = atoll(s);
    if ((s = getconfig(file, "intp_power_k")) != NULL)
         c->intp_power_k = atoll(s);
    if ((s = getconfig(file, "qos_level_num")) != NULL)
         c->qos_level_num = atoll(s);
    if ((s = getconfig(file, "worker_num_per_numa")) != NULL)
         c->worker_num_per_numa = atoll(s);
    if ((s = getconfig(file, "limit_total")) != NULL)
         c->limit_total = atoll(s);
    if ((s = getconfig(file, "limits[0]")) != NULL)
         c->limits[0] = atoll(s);
    if ((s = getconfig(file, "limits[1]")) != NULL)
         c->limits[1] = atoll(s);
    if ((s = getconfig(file, "limits[2]")) != NULL)
         c->limits[2] = atoll(s);
    if ((s = getconfig(file, "limits[3]")) != NULL)
         c->limits[3] = atoll(s);
    return 0;
}

uint64_t now_us()
{
  struct timeval tv;
  gettimeofday (&tv, NULL);
  return (uint64_t)(tv.tv_sec * 1000000 + tv.tv_usec);
}

void* thread_inotify (void *data)
{
    const char *file = "dlimitor.cfg";
    dlimitor_t *limitor = (dlimitor_t *)data;
    dlimitor_cfg_t c = {0};
    char buf[BUFSIZ] = {0};
    int fd, length, nread, wd;
    struct inotify_event *event;
    unsigned int watch_flag = IN_CLOSE_WRITE | IN_MOVE | IN_MOVED_TO | IN_IGNORED;

    if ((fd = inotify_init()) < 0) {
       fprintf(stderr, "inotify_init failed\n");
       return NULL;
    }
    while(1) {
        wd = inotify_add_watch(fd, file, watch_flag);
        if (wd < 0) {
           fprintf(stderr, "inotify_add_watch %s failed\n", file);
	   inotify_rm_watch(fd, wd);
	   continue;
        }
        length = read(fd, buf, sizeof(buf)-1);
        nread = 0;
        while(length > 0) {
            event = (struct inotify_event *)&buf[nread];
            nread += sizeof(struct inotify_event) + event->len;
            length -= sizeof(struct inotify_event) + event->len;
	    if((event->mask & (IN_MOVE | IN_CLOSE_WRITE)) && (event->len == 0) && (event->wd == wd)) {
	        read_dlimitor_config(file, &c);
	        dlimitor_update_config(limitor, &c);
	        break;
	    }
	    if((event->mask & (IN_IGNORED | IN_MOVED_TO)) && (event->wd == wd)) {
		inotify_rm_watch(fd, wd);
		sleep(1);
		break;
	    }
        }
    }
    close(fd);
    return NULL;
}

void * thread_prints (void *data)
{
    uint64_t old[COUNTER_MAX] = {0};
    dlimitor_t *limitor = (dlimitor_t *)data;
    //printf ("pid[%ld]: started thread_printf() \n", syscall (SYS_gettid));
    uint64_t t0 = now_us(), secs = 1;
    while (1) {
      memcpy(old, limitor->sum, 64);
      sleep (secs);
      dlimitor_host_stats (limitor, (now_us() - t0), old, secs);
    }
    return NULL;
}

void *
thread_function (void *data)
{
    dlimitor_t *limitor = (dlimitor_t *)data;
    int tid = syscall (SYS_gettid);
    int qos_level;
    int core_id = tid % limitor->cfg.worker_num_per_numa;
    int numa_id = core_id % NUMA_MAX;
    uint64_t qos_num = limitor->cfg.qos_level_num;
    uint64_t rnd;
#ifdef NUMA_ALLOC
    struct bitmask *mask = numa_allocate_nodemask();
    mask = numa_bitmask_clearall(mask);
    mask = numa_bitmask_setbit(mask,numa_id);
    numa_run_on_node_mask(mask);
    numa_bind(mask);
    numa_set_membind(mask);
    numa_free_nodemask(mask);
#endif
    //printf ("pid[%ld]: started thread [%d]\n", syscall (SYS_gettid), getpid());
    mt_srand(tid);
    while (1) {
        rnd = mt_rand();
        //usleep(0);
        qos_level = (rnd * tid) % qos_num;
        dlimitor_worker_update(limitor, numa_id, core_id, qos_level, 1, rnd, now_us());
    }
    return NULL;
}

int free_dlimitor_memory(dlimitor_t *limitor)
{
    uint64_t j;

#ifdef NUMA_ALLOC
    uint64_t size;
    size = sizeof(numa_update_t) + limitor->cfg.worker_num_per_numa * COUNTER_MAX * sizeof(uint64_t);
    for (j = 0; j < limitor->cfg.numa_num; j++)
        if (limitor->numas[j])
            numa_free((void *)(limitor->numas[j]), size);  
#else
    for (j = 0; j < limitor->cfg.numa_num; j++)
        if (limitor->numas[j]) free(limitor->numas[j]);     
#endif
    free(limitor);
    return 0;
}

int
main (int argc, char **argv)
{
    uint64_t i, size, thread_num = 11;
    //int cpu_num = sysconf(_SC_NPROCESSORS_ONLN);
    dlimitor_t *limitor;
    numa_update_t *numa, *numas[NUMA_MAX];
    dlimitor_cfg_t cfg = {
        32258,       /* .update_interval */
        1000000,     /* .second_ticks */
        4,           /* .sliding_power_w */
        16,          /* .intp_power_k */
        4,           /* .qos_level_num */
        NUMA_MAX,    /* .numa_num */
        thread_num,  /* .worker_num_per_numa */
        20000000,    /* .limit_total */
        {[0]=40000, [1]=20000, [2]=2000,[3]=10000}
    };

    /* malloc for dlimtor structure */
    if (posix_memalign ((void **) (&limitor), 64, sizeof (*limitor)))
        return -1;

    /* per-numa malloc for numa_update_t structure and worker counters */
    size = sizeof(*numa) + cfg.worker_num_per_numa * COUNTER_MAX * sizeof(uint64_t);
#ifdef NUMA_ALLOC
    numa_set_strict(1);
    cfg.numa_num = numa_num_configured_nodes();
    printf("numa max nodes is %ld\n", cfg.numa_num);
#endif
    for (i = 0; i < cfg.numa_num; i++) {
#ifdef NUMA_ALLOC
        if (!(numa = numa_alloc_onnode(size, i)))
            return free_dlimitor_memory(limitor);
#else
        if (posix_memalign ((void **) (&numa), 64, size))
            return free_dlimitor_memory(limitor);
#endif
        numas[i] = numa;
    }

    dlimitor_init(limitor, numas, &cfg);
    printf("created limitor\n");

    thread_num = 2 * cfg.worker_num_per_numa;
    pthread_t pid[ thread_num + 2 ];
    
    printf ("Creating %ld worker pthreads:\n", thread_num);
    for (i = 0; i < thread_num; i++)
        if (pthread_create (&pid[i], NULL, thread_function, limitor) < 0)
            return -1;
    
    printf ("Creating print pthread.\n");
    if (pthread_create (&pid[thread_num], NULL, thread_prints, limitor) < 0)
        return -1;

    printf ("Creating config-update pthread.\n");
    if (pthread_create (&pid[thread_num+1], NULL, thread_inotify, limitor) < 0)
        return -1;
    
    printf ("Now start working ...\n");
    for (i = 0; i <= thread_num+1; i++)
        pthread_join (pid[i], NULL);

    return free_dlimitor_memory(limitor);
}

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

#define NUMA_ALLOC
#ifdef NUMA_ALLOC
#include <numa.h>
#endif

#include "mtrand.h"
#include "dlimitor.h"


host_sum_t *g_limitor = NULL;
numa_sum_t *g_numa0 = NULL;
numa_sum_t *g_numa1 = NULL;
dlarray_t   g_da;
bmfilter_t *g_bmf[NUMA_MAX];


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

int read_dlimitor_config(const char *file, void *cfg, void *qos)
{
    char *s;
    dlimitor_cfg_t *c = (dlimitor_cfg_t *)cfg;
    dlimitor_qos_t *q =  (dlimitor_qos_t *)qos;
    memset(c, 0, sizeof(dlimitor_cfg_t));
    memset(q, 0, sizeof(dlimitor_qos_t));

    if ((s = getconfig(file, "update_interval")) != NULL)
         c->update_interval = atoll(s);
    if ((s = getconfig(file, "second_ticks")) != NULL)
         c->second_ticks = atoll(s);
    if ((s = getconfig(file, "sliding_power_w")) != NULL)
         c->sliding_power_w = atoll(s);
    if ((s = getconfig(file, "qos_num")) != NULL)
         q->qos_num = atoll(s);
    if ((s = getconfig(file, "limit_total")) != NULL)
         q->limit_total = atoll(s);
    if ((s = getconfig(file, "limits[0]")) != NULL)
         q->limits[0] = atoll(s);
    if ((s = getconfig(file, "limits[1]")) != NULL)
         q->limits[1] = atoll(s);
    if ((s = getconfig(file, "limits[2]")) != NULL)
         q->limits[2] = atoll(s);
    if ((s = getconfig(file, "limits[3]")) != NULL)
         q->limits[3] = atoll(s);
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
    dlimitor_qos_t q = {0};
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
	        read_dlimitor_config(file, &c, &q);
            dlarray_cfg_update(&c, &g_da);
            dlarray_qos_update(&q, &g_da, limitor);
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
    dlimitor_t *limitor = (dlimitor_t *)data;
    //printf ("pid[%ld]: started thread_printf() \n", syscall (SYS_gettid));
    while (1) {
      sleep (1);
      dlimitor_host_stats (limitor->numas[0], now_us());
    }
    return NULL;
}

void *
thread_function (void *data)
{
    dlimitor_t *limitor = (dlimitor_t *)data;
    dlimitor_qos_t *qos = (dlimitor_qos_t *)(limitor->cfg + 1);
    int tid = syscall (SYS_gettid);
    int qos_level;
    int core_id = tid % (2 * limitor->numas[0]->worker_num);
    int numa_id = core_id % NUMA_MAX;
    numa_sum_t *numa = limitor->numas[numa_id];
    uint64_t qos_num = qos->qos_num;
    uint64_t rnd;
#ifdef NUMA_ALLOC
    struct bitmask *mask = numa_allocate_nodemask();
    mask = numa_bitmask_clearall(mask);
    mask = numa_bitmask_setbit(mask,numa_id);
    numa_run_on_node_mask(mask);
    numa_bind(mask);
    numa_set_membind(mask);
    numa_free_nodemask(mask);
    numa_run_on_node(numa_id);
#endif
    //printf ("pid[%ld]: started thread [%d]\n", syscall (SYS_gettid), getpid());
    mt_srand(tid);
    core_id = core_id >> 1;
    while (1) {
        rnd = mt_rand();
        //usleep(rnd & 0xffff); /* 0xfffff = 1,048,575 */
        qos_level = (rnd * tid) % qos_num;
        dlimitor_worker_update(numa, core_id, qos_level, 1, rnd, now_us());
    }
    return NULL;
}



int
main (int argc, char **argv)
{
    int i;
    const int thread_num = 20;
    pthread_t pid[thread_num + 2];
    //int cpu_num = sysconf(_SC_NPROCESSORS_ONLN);
	uint32_t dlnum = (1U << 0);
	dlimitor_cfg_t cfg = {
		.sliding_power_w = 4,
		.second_ticks = 1000000,
		.update_interval = 1000000/((2<<4)-1),
		.flags = (LOCAL_UPDATE|PTR_BUCKET), // LOCAL_UPDATE | PTR_BUCKET
	};
	uint32_t t = 19, q = t - 5;
	dlimitor_qos_t default_qos = {
		.qos_num = 4,
		.limit_total = (1U<<t)/dlnum,            /* total limit up to 2048 Kpps */
		.limits = { [0] = 3*2*(1U<<q)/dlnum,     /* white-list-ip, limit to 3*128=384 Kpps */
		            [1] = 3*3*(1U<<q)/dlnum,     /* real-ip conn pkt, limit to 3*192=576 Kpps */
					[2] = 3*3*(1U<<q)/dlnum,     /* real-ip data pkt, limit to 3*192=576 Kpps */
					[3] = 1*24*(1U<<q)/dlnum },  /* non-real-ip, limit to 1*24*64=1536 Kpps */
	};

    
    i = numa_num_configured_nodes();
    if(numa_available() < 0 || i != NUMA_MAX){
        printf("configured numa number [%d] is not equal to expected [%d]\n", i, NUMA_MAX);
        return -1;
    }
    numa_set_strict(1);


    if ((dlnum = dlarray_init(&g_da, dlnum, thread_num, &cfg, &default_qos)) < 0)
		goto ERR_RETURN;

	for (i = 0; i < NUMA_MAX; i++)
   		if (bitmap_filter_init(&g_bmf[i], i) < 0)
			goto ERR_RETURN;

	for (i = 0; i < dlnum; i++) {
		if (dlarray_add_limitor(&g_da, i, NULL) < 0)
	    	goto ERR_RETURN;
	}
	printf("Successful add limitor [%d]\n", g_da.dlimitor_count);

	if (g_da.cfg.flags & PTR_BUCKET) {
		g_numa0 = g_da.skmem[0]->numa_buckets.p[0];
		g_numa1 = g_da.skmem[1]->numa_buckets.p[0];
	} else {
		g_numa0 = g_da.skmem[0]->numa_buckets.a;
		g_numa1 = g_da.skmem[1]->numa_buckets.a;
	}
	g_limitor = g_numa0->limitor ? g_numa0->limitor : g_numa1->limitor;
   

    


    printf ("Creating %d worker pthreads:\n", thread_num);
    for (i = 0; i < thread_num; i++)
        if (pthread_create (&pid[i], NULL, thread_function, g_limitor) < 0)
            return -1;
    
    printf ("Creating print pthread.\n");
    if (pthread_create (&pid[thread_num], NULL, thread_prints, g_limitor) < 0)
        return -1;

    printf ("Creating config-update pthread.\n");
    if (pthread_create (&pid[thread_num+1], NULL, thread_inotify, g_limitor) < 0)
        return -1;
    
    printf ("Now start working ...\n");
    for (i = 0; i <= thread_num+1; i++)
        pthread_join (pid[i], NULL);

ERR_RETURN:
    dlarray_free(&g_da);
    return 0;
}

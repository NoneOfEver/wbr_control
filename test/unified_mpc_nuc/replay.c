#define _GNU_SOURCE
#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <sys/mman.h>
#include "osqp.h"
#include "workspace.h"

static double now_us(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec * 1e-3;
}
static int compare(const void *a, const void *b) {
    double x=*(const double*)a,y=*(const double*)b;
    return (x>y)-(x<y);
}
int main(int argc,char **argv) {
    if(argc!=3 && argc!=4) return 2;
    int paced=argc==4;
    int realtime=paced && strcmp(argv[3],"rt")==0;
    FILE *f=fopen(argv[1],"rb"), *s=fopen(argv[2],"rb");
    int h[6];
    if(!f||!s||fread(h,sizeof(int),6,f)!=6) return 3;
    int n=h[0],m=h[1],nnz=h[2],frames=h[3],per=frames/3;
    size_t stride=(size_t)n+2*m+nnz;
    double *data=malloc(sizeof(double)*stride*frames);
    int *ai=malloc(sizeof(int)*nnz),*ap=malloc(sizeof(int)*(n+1));
    double *values=malloc(sizeof(double)*m),*timings=malloc(sizeof(double)*per);
    if(!data||!ai||!ap||!values||!timings) return 4;
    if(fread(data,sizeof(double),stride*frames,f)!=stride*frames ||
       fread(ai,sizeof(int),nnz,s)!=(size_t)nnz || fread(ap,sizeof(int),n+1,s)!=(size_t)(n+1)) return 5;
    fclose(f);fclose(s);
    // Prepare pages before FIFO; fail explicitly if requested settings cannot apply.
    for(int r=0;r<m;r++) values[r]=0;
    for(int k=0;k<per;k++) timings[k]=0;
    if(realtime) {
        if(mlockall(MCL_CURRENT|MCL_FUTURE)!=0) {
            fprintf(stderr,"mlockall failed: %s\n",strerror(errno));return 6;
        }
        struct sched_param parameter={.sched_priority=40};
        if(sched_setscheduler(0,SCHED_FIFO,&parameter)!=0) {
            fprintf(stderr,"SCHED_FIFO failed: %s\n",strerror(errno));return 7;
        }
    }
    struct sched_param actual;
    if(sched_getparam(0,&actual)!=0) return 8;
    fprintf(stderr,"policy=%d priority=%d cpu=%d memory_locked=%d\n",sched_getscheduler(0),actual.sched_priority,sched_getcpu(),realtime);
    for(int scenario=0;scenario<3;scenario++) {
        osqp_cold_start(&solver);
        double sum=0,max_violation=0,first=0,warm_max=0,max_wake=0;int fail=0,late=0,max_iter=0,deadline_misses=0;
        struct timespec target;
        clock_gettime(CLOCK_MONOTONIC,&target);
        for(int k=0;k<per;k++) {
            double *q=data+stride*(scenario*per+k),*l=q+n,*u=l+m,*ax=u+m;
            for(int r=0;r<m;r++) {
                if(isinf(l[r])) l[r]=copysign(OSQP_INFTY,l[r]);
                if(isinf(u[r])) u[r]=copysign(OSQP_INFTY,u[r]);
            }
            double release=0;
            if(paced) {
                target.tv_nsec+=1000000;
                if(target.tv_nsec>=1000000000) {target.tv_nsec-=1000000000;target.tv_sec++;}
                clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&target,NULL);
                release=target.tv_sec*1e6+target.tv_nsec*1e-3;
            }
            double start=now_us();
            if(paced && start-release>max_wake) max_wake=start-release;
            int err=osqp_update_data_mat(&solver,NULL,NULL,0,ax,NULL,nnz);
            err|=osqp_update_data_vec(&solver,q,l,u);
            err|=osqp_solve(&solver);
            double elapsed=now_us()-start;
            timings[k]=elapsed;sum+=elapsed;late+=elapsed>1000;
            if(k && elapsed>warm_max) warm_max=elapsed;
            if(paced && start+elapsed>release+1000) deadline_misses++;
            if(k==0) first=elapsed;
            if(solver.info->iter>max_iter) max_iter=solver.info->iter;
            if(err||solver.info->status_val!=OSQP_SOLVED) {fail++;continue;}
            for(int r=0;r<m;r++) values[r]=0;
            for(int c=0;c<n;c++) for(int j=ap[c];j<ap[c+1];j++) values[ai[j]]+=ax[j]*solver.solution->x[c];
            for(int r=0;r<m;r++) {
                double violation=fmax(l[r]-values[r],values[r]-u[r]);
                if(violation>max_violation) max_violation=violation;
            }
        }
        qsort(timings,per,sizeof(double),compare);
        printf("{\"scenario_index\":%d,\"cycles\":%d,\"mean_us\":%.3f,\"p99_us\":%.3f,\"p999_us\":%.3f,\"max_us\":%.3f,\"warm_max_us\":%.3f,\"cold_us\":%.3f,\"over_1ms\":%d,\"not_solved\":%d,\"max_iterations\":%d,\"max_constraint_violation\":%.9g,\"paced\":%d,\"max_wake_lateness_us\":%.3f,\"deadline_misses\":%d}\n",
               scenario,per,sum/per,timings[(int)(.99*(per-1))],timings[(int)(.999*(per-1))],timings[per-1],warm_max,first,late,fail,max_iter,max_violation,paced,max_wake,deadline_misses);
    }
    free(data);free(ai);free(ap);free(values);free(timings);
    return 0;
}

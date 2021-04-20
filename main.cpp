#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include "simple_test.h"

#define PERIOD_NS 1000000
#define SEC_IN_NSEC 1000000000

using namespace std;

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

static void set_latency_target(void)
{
    struct stat s;
    int err;
    int errno_;
    errno_ = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1)
    {
        std::cout << "WARN: stat /dev/cpu_dma_latency failed" << std::endl;
        return;
    }

    errno_ = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1)
    {
        std::cout << "WARN: open /dev/cpu_dma_latency" << std::endl;
        return;
    }

    errno_ = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1)
    {
        std::cout << "# error setting cpu_dma_latency to %d!" << latency_target_value << std::endl;
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}


void *ethercatThread(void *data)
{
    struct timespec ts;
    struct timespec ts2;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    double max = 0;
    double min = 2000;
    double avg = 0;
    double total =0;
    double latency = 0;
    int t_cnt = 0;
    ts.tv_sec += 1;
    while(t_cnt < 10000)
    {
        t_cnt++;
        ts.tv_nsec +=PERIOD_NS;

        while(ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        clock_gettime(CLOCK_MONOTONIC, &ts2);

        latency = (ts2.tv_nsec - ts.tv_nsec ) / 1000.0 ;
        
        // if(latency < 0)
        // {
        //     latency += SEC_IN_NSEC;
        // } ??

        if(max < latency)
           max = latency;
        
        if(min > latency)
           min = latency;

        total += latency;
        avg = total / t_cnt;
    }
    std::cout << "avg : " << avg  << "us max : " << max << "us min : "<< min   << "us"<< std::endl;
    return NULL;

}





int main(int argc, char *argv[])
{
    /*
    std::thread thread1, thread2;

    //RT THREAD FIRST!
    thread1 = std::thread(ethercatThread1);

    //EthercatElmo Management Thread
    thread2 = std::thread(ethercatThread2);
    
    sched_param sch;
    int policy;
    int priority = 39;
    pthread_getschedparam(thread1.native_handle(), &policy, &sch);

    sch.sched_priority = priority;
    if (pthread_setschedparam(thread1.native_handle(), SCHED_FIFO, &sch))
    {
        std::cout << "Failed to setschedparam: " << std::strerror(errno) << std::endl;
    }
    set_latency_target();

    thread1.join();
    thread2.join();

    std::cout << "tocabiEcat Shutdown" << std::endl;
    return 0;*/

    std::cout << "main start" << std::endl;
    struct sched_param param;
    pthread_attr_t attr, attr2;
    pthread_t thread1, thread2;

    int ret;
    int a = 500;
    //int iret1;

    set_latency_target();



 

    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread1 attributes failed\n");
        goto out;
    }

    //ret = pthread_attr_init(&attr2);
    //if (ret)
    //{
    //    printf("init pthread2 attributes failed\n");
    //    goto out;
    //}

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        goto out;
    }
    
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(4, &cpuset);

    ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    if (ret)
    {
        printf("pthread setaffinity failed\n");
        goto out;
    } 

    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        goto out;
    }

    /* Create a pthread with specified attributes */

    ret = pthread_create(&thread1, &attr, ethercatThread, (void*) &ctime);
    //ret = pthread_create(&thread2, &attr2, &ecatcheck, (void*) &ctime);
    //ret = pthread_create(&thread2, &attr2, ethercatThread2, NULL);

    //ret = pthread_create(&thread1, &attr, ethercatThread1, NULL);
    //simpletest2(argv[1]); 
    cout << "ret : " << ret <<endl;

    if (ret)
    {
        printf("create pthread failed\n");
        goto out;
            
    }
    pthread_attr_destroy(&attr);
    //pthread_attr_destroy(&attr2);
    //pthread_attr_destroy(&attr2);
    cout << "stop it" << endl;
    /* Join the thread and wait until it is done */
    ret = pthread_join(thread1, NULL);
    //ret = pthread_join(thread2, NULL);


// if (argc > 1) {

//     /* Initialize pthread attributes (default values) */
    
//     iret1 = pthread_create( &thread1, NULL, &ecatcheck, (void*) &ctime);
//     simpletest2(argv[1]); 
// }









//    int iret1;
//    int iret2;

//    std::cout << argc << std::endl;
//    if (argc > 1)
//    {
//       /* create thread to handle slave error handling in OP */
//       pthread_create( &thread1, NULL, (void *) &ecatcheck, (void*) &ctime);
//       iret1 = pthread_create( &thread1, NULL, &ecatcheck, (void*) &ctime); // (void) &ctime
//       osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
//       start cyclic part
//      iret1 = pthread_create( &thread1, NULL, &ecatcheck, (void*) &ctime);
//      simpletest2(argv[1]); 
     
//      }

//    else
//    {
//       printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
//    }
//     printf("End program\n");
   
    



out:

return ret;

}



//int32_t latency_target_value = 0;

/*
void set_latency_target(void)
{
    struct stat s;
    int err;
    int errno_;
    errno_ = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1)
    {
        std::cout << "WARN: stat /dev/cpu_dma_latency failed" << std::endl;
        return;
    }

    errno_ = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1)
    {
        std::cout << "WARN: open /dev/cpu_dma_latency" << std::endl;
        return;
    }

    errno_ = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1)
    {
        std::cout << "# error setting cpu_dma_latency to %d!" << latency_target_value << std::endl;
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);

    */
/*
 * This is a C++ version of the canonical pthread service example. It intends
 * to abstract the service management functionality and sequencing for ease
 * of use. Much of the code is left to be implemented by the student.
 *
 * Build with g++ --std=c++23 -Wall -Werror -pedantic
 * Steve Rizor 3/16/2025
 * 
 * #References used in this code:
 * This code combines parts of model code from Exercises 1 to 4,
 * along with help from LLM-based tools for C++ syntax and structure.
 */
#include <cstdint>
#include <cstdio>
#include <thread>
#include <chrono>
#include <time.h>
#include <syslog.h>

#include "Sequencer.hpp"


#define NSEC_PER_SEC (1000000000)

#define FIB_SEQ_CNT 47
#define FIB10_ITER_CNT 26000   // tuned for ~10ms
#define FIB20_ITER_CNT 56000   // tuned for ~20ms

#define FIB_TEST(seqCnt, iterCnt)            \
    for (int idx = 0; idx < iterCnt; idx++)  \
    {                                        \
        int fib0 = 1, fib1 = 0, fib = 1;      \
        for (int jdx = 0; jdx < seqCnt; jdx++)\
        {                                    \
            fib0 = fib1;                     \
            fib1 = fib;                      \
            fib = fib0 + fib1;               \
        }                                    \
    }

int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
    int dt_sec = stop->tv_sec - start->tv_sec;
    int dt_nsec = stop->tv_nsec - start->tv_nsec;

    if (dt_sec >= 0)
    {
        if (dt_nsec >= 0)
        {
            delta_t->tv_sec = dt_sec;
            delta_t->tv_nsec = dt_nsec;
        }
        else
        {
            delta_t->tv_sec = dt_sec - 1;
            delta_t->tv_nsec = NSEC_PER_SEC + dt_nsec;
        }
    }
    else
    {
        if (dt_nsec >= 0)
        {
            delta_t->tv_sec = dt_sec;
            delta_t->tv_nsec = dt_nsec;
        }
        else
        {
            delta_t->tv_sec = dt_sec - 1;
            delta_t->tv_nsec = NSEC_PER_SEC + dt_nsec;
        }
    }

    return 1;
}

void runFib10(void)
{
    static uint32_t fib10Cnt = 0;
    struct timespec start_time, end_time, delta;

    clock_gettime(CLOCK_REALTIME, &start_time);
    syslog(LOG_INFO, "fib10 started execution at %ld.%09ld seconds", start_time.tv_sec, start_time.tv_nsec);

    FIB_TEST(FIB_SEQ_CNT, FIB10_ITER_CNT);

    clock_gettime(CLOCK_REALTIME, &end_time);
    delta_t(&end_time, &start_time, &delta);

    syslog(LOG_INFO, "fib10 completed execution at %ld.%09ld seconds", end_time.tv_sec, end_time.tv_nsec);
    syslog(LOG_INFO, "fib10 execution time: %ld.%09ld seconds", delta.tv_sec, delta.tv_nsec);
    syslog(LOG_INFO, "fib10 executed %u times", ++fib10Cnt);
}

void runFib20(void)
{
    static uint32_t fib20Cnt = 0;
    struct timespec start_time, end_time, delta;

    clock_gettime(CLOCK_REALTIME, &start_time);
    syslog(LOG_INFO, "fib20 started execution at %ld.%09ld seconds", start_time.tv_sec, start_time.tv_nsec);

    FIB_TEST(FIB_SEQ_CNT, FIB20_ITER_CNT);

    clock_gettime(CLOCK_REALTIME, &end_time);
    delta_t(&end_time, &start_time, &delta);

    syslog(LOG_INFO, "fib20 completed execution at %ld.%09ld seconds", end_time.tv_sec, end_time.tv_nsec);
    syslog(LOG_INFO, "fib20 execution time: %ld.%09ld seconds", delta.tv_sec, delta.tv_nsec);
    syslog(LOG_INFO, "fib20 executed %u times", ++fib20Cnt);
}

int main()
{
    openlog("PthreadService", LOG_PID | LOG_CONS | LOG_PERROR, LOG_USER);
    syslog(LOG_INFO, "Starting sequencer with services...");
    // Example use of the sequencer/service classes:
    Sequencer sequencer{};

    sequencer.addService(runFib10, 1, 99, 20);
    sequencer.addService(runFib20, 1, 98, 50);

    sequencer.startServices();
    // todo: wait for ctrl-c or some other terminating condition
     std::this_thread::sleep_for(std::chrono::seconds(1));  // run for 5 seconds
    sequencer.stopServices();
    //std::this_thread::sleep_for(std::chrono::seconds(1));  // run for 5 seconds
    syslog(LOG_INFO, "Stopped sequencer and services.");
    closelog();
    
    return 0;
}

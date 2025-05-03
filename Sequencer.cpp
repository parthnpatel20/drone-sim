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


static void handle_sigint(int) {
    running = 0;
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



int main()
{
    openlog("Drone-sim", LOG_PID | LOG_CONS | LOG_PERROR, LOG_USER);
    syslog(LOG_INFO, "Starting sequencer with services...");

    init_drone();

    std::signal(SIGINT, handle_sigint);

    Sequencer sequencer{};

    sequencer.addService(motor_actuation_thread, 1, 99, 10);
    sequencer.addService(sensor_thread, 1, 98, 50);
    sequencer.addService(logger_thread, 1, 97, 200);

    sequencer.startServices();
    syslog(LOG_INFO, "Starting drone services...");
    while (running) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    syslog(LOG_INFO, "SIGINT received, stopping services...");
    sequencer.stopServices();

    syslog(LOG_INFO, "Stopped sequencer and services.");
    services_cleanup();
    closelog();
    
    return 0;
}

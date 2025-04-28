// File: main.c

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <pigpio.h>

// I2C / MPU6050 settings
#define I2C_BUS     "/dev/i2c-1"
#define MPU_ADDR    0x68

// Calibration & scaling
#define CAL_SAMPLES 200
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

// PWM pins & settings
#define MOTOR0_PIN  18
#define MOTOR1_PIN  19
#define MOTOR2_PIN  12
#define MOTOR3_PIN  13
#define PWM_FREQ    800   // Hz

// Shared sensor data + mutex
static pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
static float global_ax, global_ay, global_az;
static float global_gx, global_gy, global_gz;

// Logging data + mutex
static pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
static long last_sensor_interval_ns = 0;
static long last_actuation_exec_ns   = 0;

// Calibration offsets
static float accel_offset[3], gyro_offset[3];

// Running flag for clean shutdown
static volatile sig_atomic_t running = 1;
static void handle_sigint(int signum) {
    (void)signum;
    running = 0;
}

// Two’s-compliment block read from MPU6050
static int mpu_read_block(int fd, uint8_t *buf) {
    uint8_t reg = 0x3B;  // ACCEL_XOUT_H
    if (write(fd, &reg, 1) != 1) return -1;
    return (read(fd, buf, 14) == 14 ? 0 : -1);
}

// Startup calibration (assumes the board is level)
static void calibrate(int fd) {
    int32_t sum_ax=0, sum_ay=0, sum_az=0;
    int32_t sum_gx=0, sum_gy=0, sum_gz=0;
    uint8_t buf[14];

    for (int i = 0; i < CAL_SAMPLES && running; i++) {
        if (mpu_read_block(fd, buf) < 0) {
            perror("Cal read");
            exit(1);
        }
        int16_t ax = (int16_t)((buf[0]<<8) | buf[1]);
        int16_t ay = (int16_t)((buf[2]<<8) | buf[3]);
        int16_t az = (int16_t)((buf[4]<<8) | buf[5]);
        int16_t gx = (int16_t)((buf[8]<<8) | buf[9]);
        int16_t gy = (int16_t)((buf[10]<<8)| buf[11]);
        int16_t gz = (int16_t)((buf[12]<<8)| buf[13]);

        sum_ax += ax; sum_ay += ay; sum_az += az;
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        usleep(5000);
    }

    float avg_ax = sum_ax / (float)CAL_SAMPLES;
    float avg_ay = sum_ay / (float)CAL_SAMPLES;
    float avg_az = sum_az / (float)CAL_SAMPLES;

    accel_offset[0] = avg_ax;
    accel_offset[1] = avg_ay;
    accel_offset[2] = avg_az - ACCEL_SCALE;  // remove 1 g
    gyro_offset[0]  = sum_gx / (float)CAL_SAMPLES;
    gyro_offset[1]  = sum_gy / (float)CAL_SAMPLES;
    gyro_offset[2]  = sum_gz / (float)CAL_SAMPLES;

    printf("Calib Offsets: A[% .1f % .1f % .1f] G[% .1f % .1f % .1f]\n",
           accel_offset[0], accel_offset[1], accel_offset[2],
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}
/*
// Sensor thread: read & store IMU data every 50 ms
static void *sensor_thread(void *arg) {
    int fd = *(int*)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long PERIOD_NS = 50L * 1000000L;

    uint8_t buf[14];
    while (running) {
        int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (err && err != EINTR) {
            fprintf(stderr, "sensor sleep error: %s\n", strerror(err));
        }

        if (mpu_read_block(fd, buf) == 0) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            long delta_ns = (now.tv_sec - next.tv_sec) * 1000000000L
                          + (now.tv_nsec - (next.tv_nsec - PERIOD_NS));
            
            int16_t rx = (int16_t)((buf[0]<<8)|buf[1]);
            int16_t ry = (int16_t)((buf[2]<<8)|buf[3]);
            int16_t rz = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
            int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
            int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

            float ax = (rx - accel_offset[0]) / ACCEL_SCALE;
            float ay = (ry - accel_offset[1]) / ACCEL_SCALE;
            float az = (rz - accel_offset[2]) / ACCEL_SCALE;
            float gdps_x = (gx - gyro_offset[0]) / GYRO_SCALE;
            float gdps_y = (gy - gyro_offset[1]) / GYRO_SCALE;
            float gdps_z = (gz - gyro_offset[2]) / GYRO_SCALE;

            // store data
            pthread_mutex_lock(&sensor_mutex);
            global_ax = ax; global_ay = ay; global_az = az;
            global_gx = gdps_x; global_gy = gdps_y; global_gz = gdps_z;
            pthread_mutex_unlock(&sensor_mutex);

            // store interval for logger
            pthread_mutex_lock(&log_mutex);
            last_sensor_interval_ns = delta_ns;
            pthread_mutex_unlock(&log_mutex);

            // debug
            printf("Sensor: Accel[g]: % .3f % .3f % .3f | Gyro[dps]: % .2f % .2f % .2f\n",
                   ax, ay, az, gdps_x, gdps_y, gdps_z);
        }

        next.tv_nsec += PERIOD_NS;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
    }
    return NULL;
}
*/


static void *sensor_thread(void *arg) {
    int fd = *(int*)arg;
    struct timespec last, now, next;
    const long PERIOD_NS = 50L * 1000000L;  // 50 ms

    // Initialize both last and next to the current time
    clock_gettime(CLOCK_MONOTONIC, &last);
    next = last;

    uint8_t buf[14];
    while (running) {
        // Sleep until the next absolute period
        int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (err && err != EINTR) {
            fprintf(stderr, "sensor sleep error: %s\n", strerror(err));
        }

        // Stamp now and compute true delta from last
        clock_gettime(CLOCK_MONOTONIC, &now);
        long delta_ns = (now.tv_sec  - last.tv_sec)  * 1000000000L
                      + (now.tv_nsec - last.tv_nsec);

        // Read the MPU6050 block
        if (mpu_read_block(fd, buf) == 0) {
            int16_t rx = (int16_t)((buf[0]<<8)|buf[1]);
            int16_t ry = (int16_t)((buf[2]<<8)|buf[3]);
            int16_t rz = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
            int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
            int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);

            float ax    = (rx - accel_offset[0]) / ACCEL_SCALE;
            float ay    = (ry - accel_offset[1]) / ACCEL_SCALE;
            float az    = (rz - accel_offset[2]) / ACCEL_SCALE;
            float gdps_x = (gx - gyro_offset[0]) / GYRO_SCALE;
            float gdps_y = (gy - gyro_offset[1]) / GYRO_SCALE;
            float gdps_z = (gz - gyro_offset[2]) / GYRO_SCALE;

            // Store the sensor data
            pthread_mutex_lock(&sensor_mutex);
            global_ax = ax; global_ay = ay; global_az = az;
            global_gx = gdps_x; global_gy = gdps_y; global_gz = gdps_z;
            pthread_mutex_unlock(&sensor_mutex);

            // Log the true interval
            pthread_mutex_lock(&log_mutex);
            last_sensor_interval_ns = delta_ns;
            pthread_mutex_unlock(&log_mutex);

            // Debug print
            printf("Sensor: Interval=%6.2f ms | Accel[g]: % .3f % .3f % .3f "
                   "| Gyro[dps]: % .2f % .2f % .2f\n",
                   delta_ns / 1e6, ax, ay, az, gdps_x, gdps_y, gdps_z);
        }

        // Move to the next period
        last = now;
        next.tv_nsec += PERIOD_NS;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
    }
    return NULL;
}


// Clamp a value to [0,1]
static float clamp01(float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

// Motor actuation: simple P-control & PWM every 10 ms
static void *motor_actuation_thread(void *arg) {
    (void)arg;
    struct timespec next, start, end;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long PERIOD_NS = 10L * 1000000L;

    const float Kp = 0.2f;
    const float base_throttle = 0.5f;

    while (running) {
        int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (err && err != EINTR) {
            fprintf(stderr, "motor sleep error: %s\n", strerror(err));
        }
        clock_gettime(CLOCK_MONOTONIC, &start);

        pthread_mutex_lock(&sensor_mutex);
        float ax = global_ax, ay = global_ay;
        pthread_mutex_unlock(&sensor_mutex);

        // compute four thrust values
        float thr0 = clamp01(base_throttle + Kp * ax);
        float thr1 = clamp01(base_throttle - Kp * ax);
        float thr2 = clamp01(base_throttle + Kp * ay);
        float thr3 = clamp01(base_throttle - Kp * ay);

        // update hardware PWM
        gpioHardwarePWM(MOTOR0_PIN, PWM_FREQ, (unsigned)(thr0 * 1e6));
        gpioHardwarePWM(MOTOR1_PIN, PWM_FREQ, (unsigned)(thr1 * 1e6));
        gpioHardwarePWM(MOTOR2_PIN, PWM_FREQ, (unsigned)(thr2 * 1e6));
        gpioHardwarePWM(MOTOR3_PIN, PWM_FREQ, (unsigned)(thr3 * 1e6));

        clock_gettime(CLOCK_MONOTONIC, &end);
        long exec_ns = (end.tv_sec - start.tv_sec)*1000000000L
                     + (end.tv_nsec - start.tv_nsec);

        // store exec time for logger
        pthread_mutex_lock(&log_mutex);
        last_actuation_exec_ns = exec_ns;
        pthread_mutex_unlock(&log_mutex);

        next.tv_nsec += PERIOD_NS;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
    }
    return NULL;
}

// Logger thread: dumps CSV every 200 ms
static void *logger_thread(void *arg) {
    (void)arg;
    FILE *f = fopen("drone_log.csv", "w");
    if (!f) {
        perror("fopen log");
        return NULL;
    }
    fprintf(f, "time_ms,sensor_interval_ms,act_exec_ms\n");

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const long PERIOD_NS = 200L * 1000000L;

    while (running) {
        int err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
        if (err && err != EINTR) {
            fprintf(stderr, "logger sleep error: %s\n", strerror(err));
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long t_ms = now.tv_sec * 1000 + now.tv_nsec / 1000000;

        pthread_mutex_lock(&log_mutex);
        long s_ns = last_sensor_interval_ns;
        long m_ns = last_actuation_exec_ns;
        pthread_mutex_unlock(&log_mutex);

        fprintf(f, "%ld,%.3f,%.3f\n",
                t_ms, s_ns/1e6, m_ns/1e6);
        fflush(f);

        next.tv_nsec += PERIOD_NS;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
    }

    fclose(f);
    return NULL;
}

int main(void) {
    // handle Ctrl+C
    signal(SIGINT, handle_sigint);

    // Open & configure I2C
    int fd = open(I2C_BUS, O_RDWR);
    if (fd < 0) {
        perror("open i2c");
        return 1;
    }
    if (ioctl(fd, I2C_SLAVE, MPU_ADDR) < 0) {
        perror("ioctl i2c");
        close(fd);
        return 1;
    }

    // Wake MPU6050
    uint8_t wake[2] = {0x6B, 0x00};
    if (write(fd, wake, 2) != 2) {
        perror("wake sensor");
        close(fd);
        return 1;
    }

    printf("Calibrating... keep level and still\n");
    calibrate(fd);

    // Init pigpio
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio init failed\n");
        close(fd);
        return 1;
    }

    // Thread attributes
    pthread_attr_t attr;
    struct sched_param param;
    pthread_t th_sensor, th_motor, th_logger;
    int err;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    // Motor thread (prio 90)
    param.sched_priority = 90;
    pthread_attr_setschedparam(&attr, &param);
    err = pthread_create(&th_motor, &attr, motor_actuation_thread, NULL);
    if (err) { fprintf(stderr, "motor create: %s\n", strerror(err)); return 1; }

    // Sensor thread (prio 80)
    param.sched_priority = 80;
    pthread_attr_setschedparam(&attr, &param);
    err = pthread_create(&th_sensor, &attr, sensor_thread, &fd);
    if (err) { fprintf(stderr, "sensor create: %s\n", strerror(err)); return 1; }

    // Logger thread (prio 70)
    param.sched_priority = 70;
    pthread_attr_setschedparam(&attr, &param);
    err = pthread_create(&th_logger, &attr, logger_thread, NULL);
    if (err) { fprintf(stderr, "logger create: %s\n", strerror(err)); return 1; }

    // Wait until Ctrl+C
    while (running) {
        sleep(1);
    }

    printf("\nShutting down...\n");
    pthread_join(th_sensor, NULL);
    pthread_join(th_motor, NULL);
    pthread_join(th_logger, NULL);

    gpioTerminate();
    close(fd);
    return 0;
}

// File: main.c

#define _GNU_SOURCE    // for pthread attr
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

// I2C / MPU6050 settings
#define I2C_BUS       "/dev/i2c-1"
#define MPU_ADDR      0x68

// Calibration & scaling
#define CAL_SAMPLES   200
#define ACCEL_SCALE   16384.0f
#define GYRO_SCALE    131.0f

static float accel_offset[3], gyro_offset[3];

// Simple I2C block read
static int mpu_read_block(int fd, uint8_t *buf) {
    uint8_t reg = 0x3B;
    if (write(fd, &reg, 1) != 1) return -1;
    return (read(fd, buf, 14) == 14 ? 0 : -1);
}

// Startup calibration (assumes level board)
static void calibrate(int fd) {
    int32_t sum_ax=0, sum_ay=0, sum_az=0, sum_gx=0, sum_gy=0, sum_gz=0;
    uint8_t buf[14];
    for (int i = 0; i < CAL_SAMPLES; i++) {
        if (mpu_read_block(fd, buf) < 0) { perror("Cal read"); exit(1); }
        int16_t ax = (int16_t)((buf[0]<<8)|buf[1]);
        int16_t ay = (int16_t)((buf[2]<<8)|buf[3]);
        int16_t az = (int16_t)((buf[4]<<8)|buf[5]);
        int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
        int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
        int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);
        sum_ax += ax; sum_ay += ay; sum_az += az;
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        usleep(5000);
    }
    accel_offset[0] = sum_ax / (float)CAL_SAMPLES;
    accel_offset[1] = sum_ay / (float)CAL_SAMPLES;
    accel_offset[2] = sum_az / (float)CAL_SAMPLES - ACCEL_SCALE;
    gyro_offset[0]  = sum_gx / (float)CAL_SAMPLES;
    gyro_offset[1]  = sum_gy / (float)CAL_SAMPLES;
    gyro_offset[2]  = sum_gz/ (float)CAL_SAMPLES;
    printf("Calib Offsets: A[% .1f % .1f % .1f] G[% .1f % .1f % .1f]\n",
           accel_offset[0], accel_offset[1], accel_offset[2],
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

// The sensor thread: reads, calibrates, prints every 50 ms
static void *sensor_thread(void *arg) {
    int fd = *(int*)arg;
    struct timespec next, last, now;
    const long PERIOD_NS = 50L * 1000L * 1000L; // 50 ms

    // Initialize next and last
    clock_gettime(CLOCK_MONOTONIC, &next);
    last = next;

    uint8_t buf[14];
    while (1) {
        // wait until next period
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

        // read sensor
        if (mpu_read_block(fd, buf) < 0) {
            perror("MPU read");
        } else {
            // calculate actual interval
            clock_gettime(CLOCK_MONOTONIC, &now);
            long delta_ns = (now.tv_sec - last.tv_sec) * 1000000000L
                            + (now.tv_nsec - last.tv_nsec);

            // decode & print sensor as before...
            int16_t rx = (int16_t)((buf[0]<<8)|buf[1]);
            int16_t ry = (int16_t)((buf[2]<<8)|buf[3]);
            int16_t rz = (int16_t)((buf[4]<<8)|buf[5]);
            int16_t gx = (int16_t)((buf[8]<<8)|buf[9]);
            int16_t gy = (int16_t)((buf[10]<<8)|buf[11]);
            int16_t gz = (int16_t)((buf[12]<<8)|buf[13]);   // … apply offsets/scale …
            

        float ax = (rx - accel_offset[0]) / ACCEL_SCALE;
        float ay = (ry - accel_offset[1]) / ACCEL_SCALE;
        float az = (rz - accel_offset[2]) / ACCEL_SCALE;
        float gdps_x = (gx - gyro_offset[0]) / GYRO_SCALE;
        float gdps_y = (gy - gyro_offset[1]) / GYRO_SCALE;
        float gdps_z = (gz - gyro_offset[2]) / GYRO_SCALE;

        printf("[ Interval: %5.2f ms] Accel[g]: % .3f % .3f % .3f | Gyro[dps]: % .2f % .2f % .2f\n",
               delta_ns/1e6, ax, ay, az, gdps_x, gdps_y, gdps_z);       
            last = now;
        }

        // schedule next
        next.tv_nsec += PERIOD_NS;
        if (next.tv_nsec >= 1000000000L) {
            next.tv_sec++;
            next.tv_nsec -= 1000000000L;
        }
    }
    return NULL;
}


int main(void) {
    int fd = open(I2C_BUS, O_RDWR);
    if (fd < 0 || ioctl(fd, I2C_SLAVE, MPU_ADDR) < 0) {
        perror("I2C init"); return 1;
    }
    // bring sensor out of sleep
    uint8_t wake[2] = { 0x6B, 0x00 };   // PWR_MGMT_1 = 0
    if (write(fd, wake, 2) != 2) {
       perror("Failed to wake MPU6050");
       close(fd);
       return 1;
       }

    calibrate(fd);

    // pthread attributes for SCHED_FIFO prio 80
    pthread_t th;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 80;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);


    int ret = pthread_create(&th, &attr, sensor_thread, &fd);
    if (ret) {
        fprintf(stderr, "pthread_create failed: %s\n", strerror(ret));
        return 1;
    }


    pthread_join(th, NULL);
    return 0;
}

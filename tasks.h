#ifndef TASKS_H
#define TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

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
#include <syslog.h>
#include <pigpio.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define I2C_BUS     "/dev/i2c-1"
#define MPU_ADDR    0x68

#define CAL_SAMPLES 200
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

#define MOTOR0_PIN  18
#define MOTOR1_PIN  19
#define MOTOR2_PIN  12
#define MOTOR3_PIN  13
#define PWM_FREQ    800

extern pthread_mutex_t sensor_mutex;
extern float global_ax, global_ay, global_az;
extern float global_gx, global_gy, global_gz;

extern int telem_sock;
extern struct sockaddr_in telem_dst;
extern pthread_mutex_t telemetry_mutex;

extern pthread_mutex_t log_mutex;
extern long last_sensor_interval_ns;
extern long last_actuation_exec_ns;

extern float accel_offset[3], gyro_offset[3];
extern volatile sig_atomic_t running;
extern int I2Cfd;


int mpu_read_block(int fd, uint8_t *buf);
void calibrate(int fd);
void sensor_thread(void);
float clamp01(float v);
void motor_actuation_thread(void);
void logger_thread(void);
int init_drone(void);  
void services_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif
#include "tasks.h"

pthread_mutex_t sensor_mutex = PTHREAD_MUTEX_INITIALIZER;
float global_ax, global_ay, global_az;
float global_gx, global_gy, global_gz;

int telem_sock;
struct sockaddr_in telem_dst;
pthread_mutex_t telemetry_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;
long last_sensor_interval_ns;
long last_actuation_exec_ns;

float accel_offset[3], gyro_offset[3];
volatile sig_atomic_t running = 1;
int I2Cfd;

int mpu_read_block(int fd, uint8_t *buf) 
{
    uint8_t reg = 0x3B;
    if (write(fd, &reg, 1) != 1) 
    {
        syslog(LOG_ERR, "I2C write reg failed: %s", strerror(errno));
        return -1;
    }
    if (read(fd, buf, 14) != 14) 
    {
        syslog(LOG_ERR, "I2C read block failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

void calibrate(int fd) 
{
    int32_t sum_ax=0, sum_ay=0, sum_az=0;
    int32_t sum_gx=0, sum_gy=0, sum_gz=0;
    uint8_t buf[14];

    for (int i = 0; i < CAL_SAMPLES && running; i++) 
    {
        if (mpu_read_block(I2Cfd, buf) < 0) 
        {
            syslog(LOG_ERR, "Calibration read failed");
            exit(1);
        }
        int16_t ax = (buf[0]<<8)|buf[1];
        int16_t ay = (buf[2]<<8)|buf[3];
        int16_t az = (buf[4]<<8)|buf[5];
        int16_t gx = (buf[8]<<8)|buf[9];
        int16_t gy = (buf[10]<<8)|buf[11];
        int16_t gz = (buf[12]<<8)|buf[13];

        sum_ax += ax; sum_ay += ay; sum_az += az;
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        usleep(5000);
    }

    accel_offset[0] = sum_ax / (float)CAL_SAMPLES;
    accel_offset[1] = sum_ay / (float)CAL_SAMPLES;
    accel_offset[2] = sum_az / (float)CAL_SAMPLES - ACCEL_SCALE;
    gyro_offset[0]  = sum_gx / (float)CAL_SAMPLES;
    gyro_offset[1]  = sum_gy / (float)CAL_SAMPLES;
    gyro_offset[2]  = sum_gz / (float)CAL_SAMPLES;

    syslog(LOG_INFO,
        "Calib Offsets: A[%.1f %.1f %.1f] G[%.1f %.1f %.1f]",
        accel_offset[0], accel_offset[1], accel_offset[2],
        gyro_offset[0], gyro_offset[1], gyro_offset[2]
    );
}

void sensor_thread(void) 
{
    static struct timespec last = {0, 0};
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    if (last.tv_sec || last.tv_nsec) 
    {
        long delta_ns = (start.tv_sec  - last.tv_sec)  * 1000000000L + (start.tv_nsec - last.tv_nsec);
        syslog(LOG_DEBUG, "Sensor: Time since last execution = %ld us", delta_ns/1000);
        pthread_mutex_lock(&log_mutex);
        last_sensor_interval_ns = delta_ns;
        pthread_mutex_unlock(&log_mutex);
    } 
    else 
    {
        syslog(LOG_DEBUG, "Sensor: first execution");
    }    
    uint8_t buf[14];

    if (mpu_read_block(I2Cfd, buf) == 0) 
    {
        int16_t rx = (buf[0]<<8)|buf[1];
        int16_t ry = (buf[2]<<8)|buf[3];
        int16_t rz = (buf[4]<<8)|buf[5];
        int16_t gx = (buf[8]<<8)|buf[9];
        int16_t gy = (buf[10]<<8)|buf[11];
        int16_t gz = (buf[12]<<8)|buf[13];

        float ax    = (rx - accel_offset[0]) / ACCEL_SCALE;
        float ay    = (ry - accel_offset[1]) / ACCEL_SCALE;
        float az    = (rz - accel_offset[2]) / ACCEL_SCALE;
        float gdps_x = (gx - gyro_offset[0]) / GYRO_SCALE;
        float gdps_y = (gy - gyro_offset[1]) / GYRO_SCALE;
        float gdps_z = (gz - gyro_offset[2]) / GYRO_SCALE;

        pthread_mutex_lock(&sensor_mutex);
        global_ax = ax; global_ay = ay; global_az = az;
        global_gx = gdps_x; global_gy = gdps_y; global_gz = gdps_z;
        pthread_mutex_unlock(&sensor_mutex);

        pthread_mutex_lock(&log_mutex);
        last_sensor_interval_ns = start.tv_nsec - last.tv_nsec;
        pthread_mutex_unlock(&log_mutex);

        syslog(LOG_DEBUG,
            "Accel[g]=%.3f %.3f %.3f | "
            "Gyro[dps]=%.2f %.2f %.2f",
            ax, ay, az, gdps_x, gdps_y, gdps_z
        );
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    long exec_ns = (end.tv_sec  - start.tv_sec)  * 1000000000L + (end.tv_nsec - start.tv_nsec);
    syslog(LOG_DEBUG, "Sensor: Execution took %ld us", exec_ns/1000);
    last = start;
}

float clamp01(float v) 
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

void motor_actuation_thread(void) 
{
    static struct timespec last = {0, 0};
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    if (last.tv_sec || last.tv_nsec) 
    {
        long delta_ns = (start.tv_sec  - last.tv_sec)  * 1000000000L + (start.tv_nsec - last.tv_nsec);
        syslog(LOG_DEBUG, "Actuator: Time since last execution = %ld us", delta_ns/1000);
    } 
    else 
    {
        syslog(LOG_DEBUG, "Actuator: first execution");
    }

    const float Kx = 1.5f;      // roll gain
    const float Ky = 1.5f;      // pitch gain
    const float Kz = 1.5f;
    const float base_throttle = 0.3f;

    pthread_mutex_lock(&sensor_mutex);
    float ax = global_ax, ay = global_ay, az = global_az;
    pthread_mutex_unlock(&sensor_mutex);

    float thr0 = clamp01(base_throttle + Kx* ax + Ky* ay + Kz* az);  // front-left
    float thr1 = clamp01(base_throttle - Kx* ax + Ky* ay + Kz* az);  // front-right
    float thr2 = clamp01(base_throttle + Kx* ax - Ky* ay + Kz* az);  // back-left
    float thr3 = clamp01(base_throttle - Kx* ax - Ky* ay + Kz* az);  // back-right

    gpioSetPWMfrequency(MOTOR0_PIN, 800);
    gpioSetPWMfrequency(MOTOR1_PIN, 800);
    gpioSetPWMfrequency(MOTOR2_PIN, 800);
    gpioSetPWMfrequency(MOTOR3_PIN, 800);

    gpioPWM(MOTOR0_PIN, (unsigned)(thr0 * 255));   // front‑left
    gpioPWM(MOTOR1_PIN, (unsigned)(thr1 * 255));   // front‑right
    gpioPWM(MOTOR2_PIN, (unsigned)(thr2 * 255));   // back‑left
    gpioPWM(MOTOR3_PIN, (unsigned)(thr3 * 255));

    clock_gettime(CLOCK_MONOTONIC, &end);
    long exec_us = (end.tv_sec  - start.tv_sec)  * 1000000000L + (end.tv_nsec - start.tv_nsec);
    syslog(LOG_DEBUG, "Actuator: Execution took %ld us", exec_us);

    pthread_mutex_lock(&log_mutex);
    last_actuation_exec_ns = exec_us;
    pthread_mutex_unlock(&log_mutex);
    last = start;
}

void logger_thread(void) 
{
    static struct timespec last = {0, 0};
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    if (last.tv_sec || last.tv_nsec) 
    {
        long delta_ns = (start.tv_sec  - last.tv_sec)  * 1000000000L + (start.tv_nsec - last.tv_nsec);
        syslog(LOG_DEBUG, "Logger: Time since last execution = %ld us", delta_ns/1000);
    }
    else 
    {
        syslog(LOG_DEBUG, "Actuator: first execution");
    }

    FILE *f = fopen("drone_log.csv", "w");
    if (!f) 
    {
        syslog(LOG_ERR, "Failed to open log file: %s", strerror(errno));
    }
    fprintf(f, "time_ms,sensor_interval_ms,act_exec_ms\n");

    pthread_mutex_lock(&log_mutex);
    long s_ns = last_sensor_interval_ns;
    long m_ns = last_actuation_exec_ns;
    pthread_mutex_unlock(&log_mutex);
    
    float ax, ay, az, gx, gy, gz;
    pthread_mutex_lock(&sensor_mutex);
    ax = global_ax;  ay = global_ay;  az = global_az;
    gx = global_gx;  gy = global_gy;  gz = global_gz;
    pthread_mutex_unlock(&sensor_mutex);
    
    fprintf(f, "%.3f,%.3f\n", s_ns/1e6, m_ns/1e6);
    fflush(f);

    char buf[256];
    int len = snprintf(buf, sizeof(buf),
    "{\"Exec\":%ld,"
    "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
    "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}\n",
    m_ns, ax, ay, az, gx, gy, gz);

    pthread_mutex_lock(&telemetry_mutex);
    if (sendto(telem_sock, buf, len, 0,
                (struct sockaddr*)&telem_dst, sizeof(telem_dst)) < 0)
    {
        syslog(LOG_ERR, "telemetry sendto failed: %s", strerror(errno));
    }
    pthread_mutex_unlock(&telemetry_mutex);
    
    fclose(f);
    clock_gettime(CLOCK_MONOTONIC, &end);
    long exec_ns = (end.tv_sec  - start.tv_sec)  * 1000000000L + (end.tv_nsec - start.tv_nsec);
    syslog(LOG_DEBUG, "Logger: Execution took %ld us", exec_ns/1000);
    last = start;
}


int init_drone(void)
{
    I2Cfd = open(I2C_BUS, O_RDWR);
    if (I2Cfd < 0) 
    {
        syslog(LOG_ERR, "Failed to open I2C bus %s: %s", I2C_BUS, strerror(errno));
        return 1;
    }
    if (ioctl(I2Cfd, I2C_SLAVE, MPU_ADDR) < 0) 
    {
        syslog(LOG_ERR, "I2C_IOCTL failed: %s", strerror(errno));
        close(I2Cfd);
        return 1;
    }

    uint8_t wake[2] = {0x6B, 0x00};
    if (write(I2Cfd, wake, 2) != 2) 
    {
        syslog(LOG_ERR, "MPU wake failed: %s", strerror(errno));
        close(I2Cfd);
        return 1;
    }

    syslog(LOG_INFO, "Calibrating... keep level and still");
    calibrate(I2Cfd);

    if (gpioInitialise() < 0) 
    {
        syslog(LOG_ERR, "pigpio init failed");
        close(I2Cfd);
        return 1;
    }

    telem_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (telem_sock < 0) 
    {
        syslog(LOG_ERR, "Failed to create telemetry socket: %s", strerror(errno));
        exit(1);
    }

    memset(&telem_dst, 0, sizeof(telem_dst));
    telem_dst.sin_family = AF_INET;
    telem_dst.sin_port   = htons(9000);
    if (inet_pton(AF_INET, "10.0.0.184", &telem_dst.sin_addr) != 1) 
    {
        syslog(LOG_ERR, "Invalid telemetry IP");
        exit(1);
    }

    return 0;
}

void services_cleanup(void)
{
    close(telem_sock);
    gpioTerminate();
    close(I2Cfd);
}


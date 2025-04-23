// File: read_mpu6050_calibrated.c

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>

#define MPU_ADDR      0x68       // I2C address of the MPU6050
#define I2C_BUS       "/dev/i2c-1"

#define CAL_SAMPLES   200        // number of samples for calibration
#define ACCEL_SCALE   16384.0f   // LSB per g at ±2g full scale
#define GYRO_SCALE    131.0f     // LSB per °/s at ±250°/s full scale

// Offsets computed at startup
float accel_offset[3] = {0}, gyro_offset[3] = {0};

// Read 14 bytes (accel, temp, gyro) into buf
int mpu_read_block(int fd, uint8_t *buf) {
    uint8_t reg = 0x3B;  // ACCEL_XOUT_H
    if (write(fd, &reg, 1) != 1) return -1;
    if (read(fd, buf, 14) != 14)   return -1;
    return 0;
}

// Perform startup calibration (assuming board level)
void calibrate_offsets(int fd) {
    uint8_t buf[14];
    int32_t sum_ax=0, sum_ay=0, sum_az=0;
    int32_t sum_gx=0, sum_gy=0, sum_gz=0;

    for (int i = 0; i < CAL_SAMPLES; i++) {
        if (mpu_read_block(fd, buf) < 0) {
            perror("Calibration read failed");
            return;
        }

        // Sign-extend each 16-bit reading
        int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
        int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
        int16_t gy = (int16_t)((buf[10] << 8)| buf[11]);
        int16_t gz = (int16_t)((buf[12] << 8)| buf[13]);

        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;

        usleep(5000);  // 5 ms between samples
    }

    accel_offset[0] = sum_ax / (float)CAL_SAMPLES;
    accel_offset[1] = sum_ay / (float)CAL_SAMPLES;
    accel_offset[2] = sum_az / (float)CAL_SAMPLES;
    gyro_offset[0]  = sum_gx / (float)CAL_SAMPLES;
    gyro_offset[1]  = sum_gy / (float)CAL_SAMPLES;
    gyro_offset[2]  = sum_gz / (float)CAL_SAMPLES;

    printf("Calibration Offsets (raw):\n");
    printf("  Accel X=%.1f, Y=%.1f, Z=%.1f\n",
           accel_offset[0], accel_offset[1], accel_offset[2]);
    printf("  Gyro  X=%.1f, Y=%.1f, Z=%.1f\n\n",
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

int main() {
    int fd;
    // 1) Open I2C bus
    if ((fd = open(I2C_BUS, O_RDWR)) < 0) {
        perror("Opening I2C bus failed");
        return 1;
    }
    // 2) Point to MPU6050
    if (ioctl(fd, I2C_SLAVE, MPU_ADDR) < 0) {
        perror("Failed to select MPU6050");
        close(fd);
        return 1;
    }
    // 3) Wake it up (clear sleep bit)
    uint8_t wake[2] = {0x6B, 0x00};
    if (write(fd, wake, 2) != 2) {
        perror("Failed to wake MPU6050");
        close(fd);
        return 1;
    }

    // 4) Calibrate offsets
    printf("Calibrating... keep the sensor perfectly level and still.\n");
    calibrate_offsets(fd);

    // 5) Main loop: read, calibrate, scale, print
    uint8_t buf[14];
    while (1) {
        if (mpu_read_block(fd, buf) < 0) {
            perror("Sensor read failed");
            break;
        }
        // sign-extend raw readings
        int16_t ax_raw = (int16_t)((buf[0]<<8) | buf[1]);
        int16_t ay_raw = (int16_t)((buf[2]<<8) | buf[3]);
        int16_t az_raw = (int16_t)((buf[4]<<8) | buf[5]);
        int16_t temp_raw= (int16_t)((buf[6]<<8) | buf[7]);
        int16_t gx_raw = (int16_t)((buf[8]<<8) | buf[9]);
        int16_t gy_raw = (int16_t)((buf[10]<<8)| buf[11]);
        int16_t gz_raw = (int16_t)((buf[12]<<8)| buf[13]);

        // apply offsets and scale
        float ax = (ax_raw - accel_offset[0]) / ACCEL_SCALE;
        float ay = (ay_raw - accel_offset[1]) / ACCEL_SCALE;
        float az = (az_raw - accel_offset[2]) / ACCEL_SCALE;
        float gx = (gx_raw - gyro_offset[0])  / GYRO_SCALE;
        float gy = (gy_raw - gyro_offset[1])  / GYRO_SCALE;
        float gz = (gz_raw - gyro_offset[2])  / GYRO_SCALE;
        float temp = temp_raw / 340.0f + 36.53f;

        // print scaled values
        printf("Accel [g] : % .3f % .3f % .3f | ", ax, ay, az);
        printf("Gyro [°/s]: % .2f % .2f % .2f | ", gx, gy, gz);
        printf("Temp: %.2f °C\n", temp);

        sleep(1);
    }

    close(fd);
    return 0;
}

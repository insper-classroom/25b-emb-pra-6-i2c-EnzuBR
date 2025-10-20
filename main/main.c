#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <stdbool.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period
#define CLICK_ACCEL_THRESHOLD 20000 

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

#define I2C_PORT i2c0

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    const float DEAD_ZONE_ANGLE = 2.0f;
    const int16_t MOUSE_MAX_SPEED = 8;
    const float SENSITIVITY = 0.8f;

    // configuracao do I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    bool can_click = true;

    while(1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = { .axis.x = gyro[0] / 131.0f, .axis.y = gyro[1] / 131.0f, .axis.z = gyro[2] / 131.0f };
        FusionVector accelerometer = { .axis.x = acceleration[0] / 16384.0f, .axis.y = acceleration[1] / 16384.0f, .axis.z = acceleration[2] / 16384.0f };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        int16_t roll_movement = 0;
        int16_t pitch_movement = 0;

        if (euler.angle.roll > DEAD_ZONE_ANGLE || euler.angle.roll < -DEAD_ZONE_ANGLE) {
            roll_movement = (int16_t)(euler.angle.roll * SENSITIVITY);
            if (roll_movement > MOUSE_MAX_SPEED) roll_movement = MOUSE_MAX_SPEED;
            if (roll_movement < -MOUSE_MAX_SPEED) roll_movement = -MOUSE_MAX_SPEED;
        }

        if (euler.angle.pitch > DEAD_ZONE_ANGLE || euler.angle.pitch < -DEAD_ZONE_ANGLE) {
            pitch_movement = (int16_t)(euler.angle.pitch * -SENSITIVITY); 
            if (pitch_movement > MOUSE_MAX_SPEED) pitch_movement = MOUSE_MAX_SPEED;
            if (pitch_movement < -MOUSE_MAX_SPEED) pitch_movement = -MOUSE_MAX_SPEED;
        }

        putchar_raw(0xFF);
        putchar_raw(0);
        putchar_raw(roll_movement & 0xFF);
        putchar_raw((roll_movement >> 8) & 0xFF);
        
        putchar_raw(0xFF);
        putchar_raw(1);
        putchar_raw(pitch_movement & 0xFF);
        putchar_raw((pitch_movement >> 8) & 0xFF);

        if (acceleration[1] > CLICK_ACCEL_THRESHOLD && can_click) {
            can_click = false;
            
            putchar_raw(0xFF);
            putchar_raw(2);
            putchar_raw(1);
            putchar_raw(0);

        } else if (acceleration[1] < 1000) {
            can_click = true;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main() {
    stdio_init_all();

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}
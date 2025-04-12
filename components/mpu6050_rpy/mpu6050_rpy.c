#include <math.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

static const char *MPU_TAG = "mpu6050_orientation";
static mpu6050_handle_t mpu6050 = NULL;

// Complementary filter constant
#define ALPHA 0.98

static void i2c_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install error");
}

void mpu6050_init(void) {
    esp_err_t ret;

    i2c_init();
    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(mpu6050, "MPU6050 create NULL");

    ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mpu6050_wake_up(mpu6050);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void mpu6050_rpy_task(void * params){
    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    const float dt = 0.1f; // 100ms

    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MPU6050_WHO_AM_I_VAL, mpu6050_deviceid, "Wrong WHO_AM_I");

    while (1) {
        ret = mpu6050_get_acce(mpu6050, &acce);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        ret = mpu6050_get_gyro(mpu6050, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);

        // --- Accelerometer-based Roll & Pitch ---
        float accel_roll  = atan2f(acce.acce_y, acce.acce_z) * 180.0f / M_PI;
        float accel_pitch = atan2f(-acce.acce_x, sqrtf(acce.acce_y * acce.acce_y + acce.acce_z * acce.acce_z)) * 180.0f / M_PI;

        // --- Gyroscope rates (degrees per second) ---
        float gyro_roll_rate  = gyro.gyro_x;
        float gyro_pitch_rate = gyro.gyro_y;
        float gyro_yaw_rate   = gyro.gyro_z;

        // --- Complementary Filter for Roll and Pitch ---
        roll  = ALPHA * (roll + gyro_roll_rate * dt) + (1.0f - ALPHA) * accel_roll;
        pitch = ALPHA * (pitch + gyro_pitch_rate * dt) + (1.0f - ALPHA) * accel_pitch;

        // --- Yaw Integration (no correction without magnetometer) ---
        yaw += gyro_yaw_rate * dt;

        // Keep yaw within 0-360
        if (yaw >= 360.0f) yaw -= 360.0f;
        if (yaw < 0.0f) yaw += 360.0f;

        ESP_LOGI(MPU_TAG, "Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", roll, pitch, yaw);

        vTaskDelay(pdMS_TO_TICKS(dt * 1000));
    }

    mpu6050_delete(mpu6050);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelete(NULL);
}

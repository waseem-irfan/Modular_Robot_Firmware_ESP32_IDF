#ifndef MPU6050_RPY_H
#define MPU6050_RPY_H

#include "mpu6050.h"


extern mpu6050_handle_t mpu6050;

#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
// Complementary filter constant
#define ALPHA 0.99

void mpu6050_init(void);


#endif
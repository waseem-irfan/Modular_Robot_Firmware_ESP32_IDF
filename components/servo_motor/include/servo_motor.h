#include <stdio.h>
#include "driver/ledc.h"

#define SPEED_MODE      LEDC_LOW_SPEED_MODE
#define TIMER_NUM       LEDC_TIMER_0
#define DUTY_RESOLUTION LEDC_TIMER_13_BIT
#define FREQUENCY       50
#define CLK_CONFIG      LEDC_AUTO_CLK
#define CHANNEL         LEDC_CHANNEL_0
#define INTR_TYPE       LEDC_INTR_DISABLE

#define SERVO_MIN_PULSEWIDTH 1120  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 1920  // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE     90    // Maximum angle in degree up to which servo can rotate

void setup_pwm(uint8_t SERVO_PIN);

void set_servo_angle(int angle);
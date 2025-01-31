#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "servo_motor.h"

#define SERVO1_PIN       GPIO_NUM_5
#define SERVO2_PIN       GPIO_NUM_18

void app_main(){
    setup_pwm(SERVO1_PIN, LEDC_CHANNEL_0);
    setup_pwm(SERVO2_PIN, LEDC_CHANNEL_1);
    vTaskDelay(pdMS_TO_TICKS(120000));

    set_servo_speed(+100, LEDC_CHANNEL_0);  
    set_servo_speed(-100, LEDC_CHANNEL_1);
    vTaskDelay(pdMS_TO_TICKS(10000));
    set_servo_speed(0, LEDC_CHANNEL_0);  
    set_servo_speed(0, LEDC_CHANNEL_1);
}
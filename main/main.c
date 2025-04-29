#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "servo_motor.h"

#define SERVO1_PIN       GPIO_NUM_12
#define SERVO2_PIN       GPIO_NUM_27

void app_main(){
    setup_pwm(SERVO1_PIN, LEDC_CHANNEL_0);
    setup_pwm(SERVO2_PIN, LEDC_CHANNEL_1);

    vTaskDelay(pdMS_TO_TICKS(60000));   

    // set_servo_speed(+25, LEDC_CHANNEL_0);  // left
    // set_servo_speed(-25, LEDC_CHANNEL_1);  //right
    // vTaskDelay(pdMS_TO_TICKS(20000));
    // set_servo_speed(0, LEDC_CHANNEL_0);  
    // set_servo_speed(0, LEDC_CHANNEL_1);
    // vTaskDelay(pdMS_TO_TICKS(5000));
    
    // set_servo_speed(+25, LEDC_CHANNEL_0);  // left
    // set_servo_speed(-27, LEDC_CHANNEL_1);  //right
    // vTaskDelay(pdMS_TO_TICKS(20000));
    // set_servo_speed(0, LEDC_CHANNEL_0);  
    // set_servo_speed(0, LEDC_CHANNEL_1);
    // vTaskDelay(pdMS_TO_TICKS(5000));

    // set_servo_speed(+25, LEDC_CHANNEL_0);  // left
    // set_servo_speed(-30, LEDC_CHANNEL_1);  //right
    // vTaskDelay(pdMS_TO_TICKS(20000));
    // set_servo_speed(0, LEDC_CHANNEL_0);  
    // set_servo_speed(0, LEDC_CHANNEL_1);
    // vTaskDelay(pdMS_TO_TICKS(5000));

    // set_servo_speed(+27, LEDC_CHANNEL_0);  // left
    // set_servo_speed(-25, LEDC_CHANNEL_1);  //right
    // vTaskDelay(pdMS_TO_TICKS(20000));
    // set_servo_speed(0, LEDC_CHANNEL_0);  
    // set_servo_speed(0, LEDC_CHANNEL_1);
    // vTaskDelay(pdMS_TO_TICKS(5000));

    set_servo_speed(+35, LEDC_CHANNEL_0);  // left
    set_servo_speed(-25, LEDC_CHANNEL_1);  //right
    vTaskDelay(pdMS_TO_TICKS(20000));
    set_servo_speed(+40, LEDC_CHANNEL_0);  // left
    set_servo_speed(-25, LEDC_CHANNEL_1);  //right
    vTaskDelay(pdMS_TO_TICKS(20000));
    set_servo_speed(0, LEDC_CHANNEL_0);  
    set_servo_speed(0, LEDC_CHANNEL_1);
    vTaskDelay(pdMS_TO_TICKS(5000));

    set_servo_speed(0, LEDC_CHANNEL_0);  
    set_servo_speed(0, LEDC_CHANNEL_1);

}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "servo_motor.h"

#define SERVO_PIN_1       GPIO_NUM_5
#define SERVO_PIN_2       GPIO_NUM_18

void app_main(){
    setup_pwm(SERVO_PIN_1);
    setup_pwm(SERVO_PIN_2);

    while(1){
        set_servo_speed(0);   // Stop
        vTaskDelay(pdMS_TO_TICKS(2000));

        set_servo_speed(25);  // Slow forward
        vTaskDelay(pdMS_TO_TICKS(2000));

        set_servo_speed(50);  // Medium forward
        vTaskDelay(pdMS_TO_TICKS(2000));

        set_servo_speed(100); // Fast forward
        vTaskDelay(pdMS_TO_TICKS(2000));

        set_servo_speed(-100); // Fast backward
        vTaskDelay(pdMS_TO_TICKS(2000));

        set_servo_speed(-50); // Medium backward
        vTaskDelay(pdMS_TO_TICKS(2000));

        set_servo_speed(-25);  // Slow Backward
        vTaskDelay(pdMS_TO_TICKS(2000));

        set_servo_speed(0);   // Stop
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "servo_motor.h"

#define SERVO_PIN       GPIO_NUM_5

void app_main(){
    setup_pwm(SERVO_PIN);

    while(1){
        set_servo_angle(90);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
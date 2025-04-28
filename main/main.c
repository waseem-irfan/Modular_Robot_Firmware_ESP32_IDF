#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"

bdc_motor_handle_t mtr1;
bdc_motor_handle_t mtr2;
uint32_t speed = 400;

void move_forward(){
    set_speed_direction(mtr1, speed, 1);
    set_speed_direction(mtr2, speed, 1);
    // vTaskDelay(pdMS_TO_TICKS(100));
}

void move_backward(){
    set_speed_direction(mtr1, speed, 0);
    set_speed_direction(mtr2, speed, 0);
}

void move_left(){
    set_speed_direction(mtr1, speed, 1);
    set_speed_direction(mtr2, speed, 0);
}

void move_right(){
    set_speed_direction(mtr1, speed, 0);
    set_speed_direction(mtr2, speed, 1);
}
void app_main(void){
    mtr1 = start_motor(GPIO_NUM_32, GPIO_NUM_13, 25000);
    mtr2 = start_motor(GPIO_NUM_2, GPIO_NUM_4, 25000);
    vTaskDelay(pdMS_TO_TICKS(10000));
    while(1){
        move_backward();
            vTaskDelay(pdMS_TO_TICKS(10));
    }
    

}
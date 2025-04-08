#include <stdio.h>
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void){
    bdc_motor_handle_t motor = start_motor(19, 21, 25000);
    while (1)
    {
        /* code */
        set_speed_direction(motor, 300, false);
            vTaskDelay(pdMS_TO_TICKS(10));

    }
    
}
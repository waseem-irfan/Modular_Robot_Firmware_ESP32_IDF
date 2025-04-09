#include <stdio.h>
#include "motor.h"
#include "pulse.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void){
    bdc_motor_handle_t motor = start_motor(19, 21, 25000);
    pcnt_unit_handle_t pcnt_unit = setup_pcnt_encoder(5, 18, 2200, -2200);
    
    // Start the motor at desired speed and direction
    set_speed_direction(motor, 200, true);

    while (1) {
        int pulse = get_encoder_pulses(pcnt_unit);  // Read encoder count
        printf("Pulse: %d\n", pulse); 
        // vTaskDelay(pdMS_TO_TICKS(100));  // Read every 100 ms

        // Optional stop condition:
        if (pulse >= 2195) {
            set_speed_direction(motor, 0, true);  // Stop motor
            printf("Target reached: %d pulses\n", pulse);
            break;
        }
    }
}

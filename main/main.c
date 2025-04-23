#include <stdio.h>
#include "pid_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "mpu6050_rpy.h"

pid_ctrl_block_handle_t pid;
pcnt_unit_handle_t pcnt;
bdc_motor_handle_t motor;

void app_main(void){
    // mpu6050_init();

    pcnt = setup_pcnt_encoder(GPIO_NUM_5, GPIO_NUM_18, 15000, -15000);

    // 2. Initialize motor
    motor = start_motor(GPIO_NUM_19, GPIO_NUM_21, 25000); // 1kHz PWM

    // 3. Initialize PID controller (already shown above)
    ESP_ERROR_CHECK(pid_cfg_init(1.1f, 0.33f, 0.02f));
    // 4. Create control task
    xTaskCreate(control_loop_task, "pid_control", 4096, NULL, 5, NULL);
    // xTaskCreate(mpu6050_rpy_task,"MPU_6050 Task", 1024*2, NULL, 2, NULL);
}
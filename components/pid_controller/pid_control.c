#include <math.h>
#include <stdbool.h>
#include "pid_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "pulse.h"
#include "esp_log.h"
#include <sys/param.h>

#define TARGET_POSITION  15000 // Desired encoder pulses
#define PID_INTERVAL_MS  10   // 10 ms control loop
#define POSITION_DEADBAND 2

static const char *PID_TAG = "pid_ctrl";

extern pid_ctrl_block_handle_t pid;
extern pcnt_unit_handle_t pcnt;
extern bdc_motor_handle_t motor;

esp_err_t pid_cfg_init(float kp, float kd, float ki){
    esp_err_t err;
    pid_ctrl_config_t cfg = {
        .init_param = {
            .kp = kp,
            .kd = kd,
            .ki = ki,
            .max_output = 10000,   // max speed value
            .min_output = -10000,  // min (reverse) speed
            .max_integral = 1000,
            .min_integral = -1000,
            .cal_type = PID_CAL_TYPE_POSITIONAL  // <<< Use this for accurate positioning not incremental method
        }
    }; 
    err = pid_new_control_block(&cfg,&pid);
    ESP_LOGI(PID_TAG,"PID Controller Configured Successfully...");
    return err;
}

// Creating Task for PID control loop

void control_loop_task(void *param) {
    int target_position = TARGET_POSITION;
    float control_output = 0;

    while (1) {
        int current_position = get_encoder_pulses(pcnt);
        int position_error = target_position - current_position;

        // Add deadband zone to prevent jittering
        if (abs(position_error) < POSITION_DEADBAND) {
            set_speed_direction(motor, 0, true); // Stop motor
            // Optional: lock position to target to prevent small error accumulation
            // current_position = target_position;
            vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
            continue;
        }

        // PID computation
        pid_compute(pid, (float)position_error, &control_output);

        // Set direction and speed
        bool forward = control_output >= 0;
        uint32_t speed = (uint32_t)fabs(control_output);
        speed = MIN(speed, 400); // Clamp to max speed
        if (speed < 100) speed = 100;
        set_speed_direction(motor, speed, forward);
        ESP_LOGI(PID_TAG,"PID_PULSE: %d",current_position);
        vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}
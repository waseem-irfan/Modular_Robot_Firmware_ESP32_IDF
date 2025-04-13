#include "pid_ctrl.h"

#define TARGET_POSITION  1000 // Desired encoder pulses
#define PID_INTERVAL_MS  10   // 10 ms control loop

static const char *TAG = "pid_ctrl";

pid_ctrl_block_handle_t pid;

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
            .cal_type = PID_CAL_TYPE_POSITIONAL  // <<< Use this for accurate positioning
        }
    }; 
    err = pid_new_control_block(&cfg,&pid);
    return err;
}

// Creating Task for PID control loop

void control_loop_task(void * param){
    
}
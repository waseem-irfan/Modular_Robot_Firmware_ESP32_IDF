#ifndef PID_CONTROL_H
#define PID_CONTROL_H
#include "esp_err.h"
#include "motor.h"
#include "pulse.h"
#include "pid_ctrl.h"

esp_err_t pid_cfg_init(float kp, float kd, float ki);
void control_loop_task(void *param);

#endif
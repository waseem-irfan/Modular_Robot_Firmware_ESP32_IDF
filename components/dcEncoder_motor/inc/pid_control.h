#ifndef PID_CONTROL_H
#define PID_CONTROL_H
#include "esp_err.h"

esp_err_t pid_cfg_init(float kp, float kd, float ki);

#endif
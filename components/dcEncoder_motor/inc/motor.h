#ifndef MOTOR_H
#define MOTOR_H

#include "driver/gpio.h"
#include "bdc_motor.h"

bdc_motor_handle_t start_motor(gpio_num_t pwm_a, gpio_num_t pwm_b, uint32_t freq);
void set_speed_direction(bdc_motor_handle_t motor,uint32_t speed, bool forward);

#endif
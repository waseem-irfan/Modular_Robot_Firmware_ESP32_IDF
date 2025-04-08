#ifndef PULSE_H
#define PULSE_H
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

pcnt_unit_handle_t setup_pcnt_encoder(gpio_num_t enc_a, gpio_num_t enc_b, uint32_t high_limit, uint32_t low_limit);
int get_encoder_pulses(pcnt_unit_handle_t pcnt_unit);

#endif
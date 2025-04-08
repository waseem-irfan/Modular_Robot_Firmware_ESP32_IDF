#include "bdc_motor.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#define MOTOR_TAG "DC_MOTOR"

bdc_motor_handle_t start_motor(gpio_num_t pwm_a, gpio_num_t pwm_b, uint32_t freq){
    ESP_LOGI(MOTOR_TAG, "Create DC motor");
    bdc_motor_config_t motor_cfg = {
        .pwm_freq_hz = freq,
        .pwma_gpio_num = pwm_a,
        .pwmb_gpio_num = pwm_b
    };

    bdc_motor_mcpwm_config_t mcpwm_cfg = {
        .group_id = 0,
        .resolution_hz = 10000000 // 10MHz, 1tick = 0.1us
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_cfg,&mcpwm_cfg,&motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    return motor;
}

void set_speed_direction(bdc_motor_handle_t motor,uint32_t speed, bool forward){
    if(forward){
        ESP_ERROR_CHECK(bdc_motor_forward(motor));
    }
    else{
        ESP_ERROR_CHECK(bdc_motor_reverse(motor));
    }
    ESP_ERROR_CHECK(bdc_motor_set_speed(motor,speed));
    /*
    Valid speed range = resolution_hz / pwm_freq;
    */
}


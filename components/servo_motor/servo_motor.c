#include "servo_motor.h"

void setup_pwm(uint8_t SERVO_PIN, ledc_channel_t channel) {
    // Configure the LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = SPEED_MODE,
        .timer_num        = TIMER_NUM,
        .duty_resolution  = DUTY_RESOLUTION,
        .freq_hz          = FREQUENCY,
        .clk_cfg          = CLK_CONFIG
    };
    ledc_timer_config(&ledc_timer);

    // Configure the LEDC channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = SPEED_MODE,
        .channel        = channel,
        .timer_sel      = TIMER_NUM,
        .intr_type      = INTR_TYPE,
        .gpio_num       = SERVO_PIN,
        .duty           = 0, // Initial duty cycle
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void set_servo_angle(int angle) 
{
    // Convert angle to duty cycle
    int pulsewidth = SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle) / SERVO_MAX_DEGREE;
    int duty = (pulsewidth * 8192) / 20000; // 8192 is 2^13 for 13-bit resolution
    ledc_set_duty(SPEED_MODE, CHANNEL, duty);
    ledc_update_duty(SPEED_MODE, CHANNEL);
}

void set_servo_speed(int speed, ledc_channel_t channel)
{
    int pulsewidth = 1500 + (speed * 4); // Scale speed to PWM range

    // Clamp pulse width to valid range (1120µs to 1920µs)
    if (pulsewidth > SERVO_MAX_PULSEWIDTH) pulsewidth = SERVO_MAX_PULSEWIDTH;
    if (pulsewidth < SERVO_MIN_PULSEWIDTH) pulsewidth = SERVO_MIN_PULSEWIDTH;

    int duty = (pulsewidth * 8192) / 20000; // Convert pulse width to duty cycle
    ledc_set_duty(SPEED_MODE, channel, duty);
    ledc_update_duty(SPEED_MODE, channel);
}
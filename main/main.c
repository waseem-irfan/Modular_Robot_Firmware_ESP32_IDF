#include <stdio.h>

#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
// #define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              19
#define BDC_MCPWM_GPIO_B              21

#define BDC_ENCODER_GPIO_A            5
#define BDC_ENCODER_GPIO_B            18
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED          400  // expected motor speed, in the pulses counted by the rotary encoder

void app_main(void){
    
}
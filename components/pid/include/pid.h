#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"

typedef struct {
    // PID gains
    float kp;  
    float ki;
    float kd;
    
    // PWM configuration
    int pwm_pin;
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t timer_num;        
    ledc_channel_t ledc_channel;
    ledc_clk_cfg_t ledc_clk_cfg;
    uint32_t frequency;
    
    // PID state variables
    float integral;
    float prev_error;
    int64_t last_update_time_us;
    bool enabled;
    bool prev_enabled;
    
    // Output limits
    float integral_max;  // Anti-windup limit
    uint32_t max_duty;   // Maximum duty cycle value
} pid_controller_t;

void pid_init(pid_controller_t *pid);
int pid_update(pid_controller_t *pid, float setpoint, float measured);
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);
void pid_reset(pid_controller_t *pid);
void pid_set_enabled(pid_controller_t *pid, bool enabled);
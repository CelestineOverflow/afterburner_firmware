#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/ledc.h"

typedef struct {
    float kp;  
    float ki;
    float kd;
    int pwm_pin;
    ledc_mode_t speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_channel_t ledc_channel;
    ledc_clk_cfg_t ledc_clk_cfg;
    uint32_t frequency;
} pid_controller_t;


void pid_init(pid_controller_t *pid);
float pid_compute(pid_controller_t *pid, float setpoint, float measured, float dt);
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

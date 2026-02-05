#include "pid.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <math.h>
#include "driver/gpio.h"

static const char *TAG = "PID";

void pid_init(pid_controller_t *pid) {
    // First, reset the GPIO to a known state
    gpio_reset_pin(pid->pwm_pin);
    
    // Calculate max duty based on resolution
    pid->max_duty = (1 << pid->duty_resolution) - 1;
    
    // Initialize PID state
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_update_time_us = 0;
    pid->enabled = false;
    pid->prev_enabled = false;
    
    // Set default integral limit if not set
    if (pid->integral_max == 0.0f) {
        pid->integral_max = pid->max_duty * 0.5f;  // Default to 50% of max duty
    }
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = pid->speed_mode,
        .duty_resolution = pid->duty_resolution,
        .timer_num = pid->timer_num,  // ← Use timer_num instead of ledc_channel
        .freq_hz = pid->frequency,  
        .clk_cfg = pid->ledc_clk_cfg,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "LEDC timer configured OK");
    
    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = pid->pwm_pin,
        .speed_mode = pid->speed_mode,
        .channel = pid->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = pid->timer_num,  // ← Use timer_num
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Start with 0 duty
    ledc_set_duty(pid->speed_mode, pid->ledc_channel, 0);
    ledc_update_duty(pid->speed_mode, pid->ledc_channel);
    
    ESP_LOGI(TAG, "PID controller initialized on pin %d", pid->pwm_pin);
}

int pid_update(pid_controller_t *pid, float setpoint, float measured) {
    // Get current time in microseconds
    int64_t current_time_us = esp_timer_get_time();
    
    // Check if controller was just disabled
    if (pid->prev_enabled && !pid->enabled) {
        // Immediately turn off heater
        ledc_set_duty(pid->speed_mode, pid->ledc_channel, 0);
        ledc_update_duty(pid->speed_mode, pid->ledc_channel);
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        ESP_LOGI(TAG, "PID disabled - heater off");
    }
    
    pid->prev_enabled = pid->enabled;
    
    // If disabled, do nothing more
    if (!pid->enabled) {
        return 0;
    }
    
    // Calculate dt in seconds
    float dt;
    if (pid->last_update_time_us == 0) {
        // First run - use a reasonable default (e.g., 100ms)
        dt = 0.1f;
    } else {
        dt = (current_time_us - pid->last_update_time_us) / 1000000.0f;
        
        // Sanity check on dt
        if (dt <= 0.0f || dt > 10.0f) {
            ESP_LOGW(TAG, "Invalid dt: %f, using 0.1s", dt);
            dt = 0.1f;
        }
    }
    
    pid->last_update_time_us = current_time_us;
    
    // Calculate error
    float error = setpoint - measured;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    
    // Clamp integral to prevent windup
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    
    // Store error for next iteration
    pid->prev_error = error;
    
    // Calculate total output
    float output = p_term + i_term + d_term;
    
    // Clamp output to valid duty cycle range [0, max_duty]
    if (output < 0.0f) {
        output = 0.0f;
    } else if (output > pid->max_duty) {
        output = pid->max_duty;
    }
    
    // Convert to integer duty cycle
    uint32_t duty = (uint32_t)output;
    
    // Set PWM duty cycle
    ledc_set_duty(pid->speed_mode, pid->ledc_channel, duty);
    ledc_update_duty(pid->speed_mode, pid->ledc_channel);
    
    return duty;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    ESP_LOGI(TAG, "PID gains updated: Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
}

void pid_reset(pid_controller_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_update_time_us = 0;
    ESP_LOGI(TAG, "PID state reset");
}

void pid_set_enabled(pid_controller_t *pid, bool enabled) {
    pid->enabled = enabled;
    
    // If disabling, immediately turn off
    if (!enabled) {
        ledc_set_duty(pid->speed_mode, pid->ledc_channel, 0);
        ledc_update_duty(pid->speed_mode, pid->ledc_channel);
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        ESP_LOGI(TAG, "PID explicitly disabled - heater off");
    } else {
        ESP_LOGI(TAG, "PID enabled");
    }
}
#include "pid.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "PID";

void pid_init(pid_controller_t *pid) {
    // First, reset the GPIO to a known state
    gpio_reset_pin(pid->pwm_pin);
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = pid->speed_mode,
        .duty_resolution = pid->duty_resolution,
        .timer_num = pid->ledc_channel,
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
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ledc_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err));
        return;
    }
    
    // Start with 0 duty
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    
}



#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_timer.h"
// User components
#include "led_control.h"
#include "ina260.h"
#include "ads1220.h"
#include "max31865.h"
#include "pd.h"
#include "stdatomic.h"
#include "cJSON.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"

static const char *TAG = "HEATER";

// APA102 Like LED
#define LED_CLOCK 4
#define LED_DATA_IN 5
#define NUM_LEDS 1
led_ctrl_t status_led = {
    .clk_pin = LED_CLOCK,
    .data_pin = LED_DATA_IN,
};

// Power Delivery TPS25730
#define PD_POWER_ON 6
#define PD_ADDRESS 0x20

// TPS25730 Register Addresses
#define TPS25730_REG_ACTIVE_CONTRACT_PDO 0x34 // Length 6
#define TPS25730_REG_ACTIVE_CONTRACT_RDO 0x35 // Length 4

// Heater Mosfet PWM
#define HEATER_PWM 7

// I2C Pins
#define SDA 18
#define SCL 19
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 100
static SemaphoreHandle_t i2c_bus_mutex = NULL;

// SPI
#define MOSI 22
#define MISO 23
#define SCK 21
static SemaphoreHandle_t spi_bus_mutex = NULL;

// MAX31865
#define TEMP_CS 20
#define TEMP_DATA_READY 3

// =============================================================================
// PID Controller Configuration
// =============================================================================
#define PID_MAX_DUTY_PERCENT    50.0f   // Maximum duty cycle percentage
#define PID_MAX_DUTY            ((uint32_t)(255 * PID_MAX_DUTY_PERCENT / 100.0f))
#define PID_MAX_TARGET_TEMP     50.0f   // Maximum allowed target temperature
#define PID_MIN_TARGET_TEMP     20.0f   // Minimum target temperature
#define PID_CONTROL_INTERVAL_MS 100     // PID update interval
#define PID_INTEGRAL_MAX        100.0f  // Anti-windup limit

// Temperature control parameters (can be modified via serial)
static float target_temperature = 25.0f;
static float kp = 5.0f;
static float ki = 0.1f;
static float kd = 2.0f;

// PID state
static float pid_integral = 0.0f;
static float pid_prev_error = 0.0f;
static int64_t pid_last_time_us = 0;
static uint32_t pid_current_duty = 0;
static atomic_bool heater_enabled = false;
static atomic_bool pid_active = false;

// PWM mode flag - if false, use direct GPIO control for testing
static bool use_pwm_mode = true;
static bool ledc_initialized = false;

// =============================================================================
// Global state variables
// =============================================================================
atomic_int acnt;

ina260_t power_monitor = {
    .i2c_port = 0,
    .i2c_addr = 0x40,
    .alert_pin = 15, 
};

ads1220_t loadcell = {
    .spi_host = SPI2_HOST,
    .cs_pin = 17,
    .drdy_pin = 16,
    .clock_hz = 1000000,
};

max31865_t rtd = {
    .spi_host = SPI2_HOST,
    .cs_pin = 20,
    .rref = 430.0f,
    .rtd_nominal = 100.0f,
    .three_wire = false,
    .clock_hz = 1000000,
};

pd_t pd_device = {
    .i2c_port = 0,
    .i2c_addr = PD_ADDRESS,
};

// Forward declaration
static void send_response(const char *cmd, bool success, const char *message);

void setupHeater()
{
    // First, reset the GPIO to a known state
    gpio_reset_pin(HEATER_PWM);
    
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 100,  // Lower frequency for heater, easier to debug
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "LEDC timer configured OK");
    
    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = HEATER_PWM,
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
    ESP_LOGI(TAG, "LEDC channel configured OK on GPIO %d", HEATER_PWM);
    
    ledc_initialized = true;
    
    // Start with 0 duty
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Set heater duty cycle with safety limits
static void heater_set_duty(uint32_t duty)
{
    // Clamp to maximum allowed duty cycle
    if (duty > PID_MAX_DUTY) {
        duty = PID_MAX_DUTY;
    }
    
    pid_current_duty = duty;
    
    if (use_pwm_mode && ledc_initialized) {
        esp_err_t err1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        esp_err_t err2 = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        if (err1 != ESP_OK || err2 != ESP_OK) {
            ESP_LOGE(TAG, "LEDC set duty failed: %s / %s", 
                     esp_err_to_name(err1), esp_err_to_name(err2));
        }
    }
}

// Turn off heater completely
static void heater_off(void)
{
    pid_current_duty = 0;
    
    if (use_pwm_mode && ledc_initialized) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        gpio_set_level(HEATER_PWM, 0);
    }
}

// Reset PID state
static void pid_reset(void)
{
    pid_integral = 0.0f;
    pid_prev_error = 0.0f;
    pid_last_time_us = esp_timer_get_time();
}

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void spi_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(
        spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO)
    );
}

// Temperature state
static volatile float latest_temp = 0.0f;
static atomic_bool temp_data_valid = false;
static QueueHandle_t temp_ready_queue = NULL;

// Loadcell state
static volatile int32_t latest_loadcell_raw = 0;
static atomic_bool loadcell_data_valid = false;
static QueueHandle_t loadcell_ready_queue = NULL;

// INA260 state
static volatile int32_t latest_voltage_mv = 0;
static volatile int32_t latest_current_ma = 0;
static volatile int32_t latest_power_mw = 0;
static atomic_bool ina260_data_valid = false;

// PD state
static volatile uint16_t latest_pd_voltage_mv = 0;
static atomic_bool pd_data_valid = false;

// JSON reporting mutex
static SemaphoreHandle_t json_print_mutex = NULL;

static void print_json(cJSON *json)
{
    if (json == NULL) return;
    
    char *json_str = cJSON_PrintUnformatted(json);
    if (json_str != NULL)
    {
        if (xSemaphoreTake(json_print_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            printf("%s\n", json_str);
            xSemaphoreGive(json_print_mutex);
        }
        cJSON_free(json_str);
    }
    cJSON_Delete(json);
}

static void report_temperature_json(float temp_c)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "temperature");
    cJSON_AddNumberToObject(json, "value", temp_c);
    cJSON_AddStringToObject(json, "unit", "C");
    cJSON_AddNumberToObject(json, "timestamp_ms", esp_timer_get_time() / 1000);
    print_json(json);
}

static void report_loadcell_json(int32_t raw)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "loadcell");
    cJSON_AddNumberToObject(json, "raw", raw);
    cJSON_AddNumberToObject(json, "timestamp_ms", esp_timer_get_time() / 1000);
    print_json(json);
}

static void report_ina260_json(int32_t voltage_mv, int32_t current_ma, int32_t power_mw)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "ina260");
    cJSON_AddNumberToObject(json, "voltage_mv", voltage_mv);
    cJSON_AddNumberToObject(json, "current_ma", current_ma);
    cJSON_AddNumberToObject(json, "power_mw", power_mw);
    cJSON_AddNumberToObject(json, "timestamp_ms", esp_timer_get_time() / 1000);
    print_json(json);
}

static void report_pd_json(uint16_t voltage_mv)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "pd");
    cJSON_AddNumberToObject(json, "voltage_mv", voltage_mv);
    cJSON_AddNumberToObject(json, "timestamp_ms", esp_timer_get_time() / 1000);
    print_json(json);
}

static char temp_str[16];
// static char duty_str[16];

static void report_pid_status_json(void)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "pid_status");
    cJSON_AddBoolToObject(json, "enabled", atomic_load(&heater_enabled));
    cJSON_AddBoolToObject(json, "active", atomic_load(&pid_active));
    // cJSON_AddBoolToObject(json, "pwm_mode", use_pwm_mode);
    // cJSON_AddBoolToObject(json, "ledc_init", ledc_initialized);
    snprintf(temp_str, sizeof(temp_str), "%.2f", target_temperature);
    cJSON_AddStringToObject(json, "target_temp", temp_str);

    // Format current temperature to 2 decimal places
    snprintf(temp_str, sizeof(temp_str), "%.2f", latest_temp);
    cJSON_AddStringToObject(json, "current_tem22p", temp_str);
    cJSON_AddNumberToObject(json, "duty", pid_current_duty);
    cJSON_AddNumberToObject(json, "duty_percent", (pid_current_duty * 100.0f) / 255.0f);
    cJSON_AddNumberToObject(json, "max_duty_percent", PID_MAX_DUTY_PERCENT);
    // cJSON_AddNumberToObject(json, "kp", kp);
    // cJSON_AddNumberToObject(json, "ki", ki);
    // cJSON_AddNumberToObject(json, "kd", kd);
    // cJSON_AddNumberToObject(json, "integral", pid_integral);
    // cJSON_AddNumberToObject(json, "pd_voltage_mv", latest_pd_voltage_mv);
    // INA
    cJSON_AddNumberToObject(json, "voltage_mv", latest_voltage_mv);
    cJSON_AddNumberToObject(json, "current_ma", latest_current_ma);
    cJSON_AddNumberToObject(json, "power_mw", latest_power_mw);

    // cJSON_AddNumberToObject(json, "timestamp_ms", esp_timer_get_time() / 1000);
    print_json(json);
}

static void send_response(const char *cmd, bool success, const char *message)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "response");
    cJSON_AddStringToObject(json, "cmd", cmd);
    cJSON_AddBoolToObject(json, "success", success);
    if (message) {
        cJSON_AddStringToObject(json, "message", message);
    }
    cJSON_AddNumberToObject(json, "timestamp_ms", esp_timer_get_time() / 1000);
    print_json(json);
}

// ISR handlers
static void IRAM_ATTR temp_ready_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gpio_intr_disable(TEMP_DATA_READY);
    xQueueSendFromISR(temp_ready_queue, &gpio_num, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void IRAM_ATTR loadcell_ready_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gpio_intr_disable(loadcell.drdy_pin);
    xQueueSendFromISR(loadcell_ready_queue, &gpio_num, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void update_loadcell_state(void *arg)
{
    uint32_t gpio_num;
    
    while (1)
    {
        if (xQueueReceive(loadcell_ready_queue, &gpio_num, portMAX_DELAY))
        {
            if (xSemaphoreTake(spi_bus_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                int32_t raw;
                esp_err_t err = ads1220_read_raw(&loadcell, &raw);
                xSemaphoreGive(spi_bus_mutex);
                
                if (err == ESP_OK)
                {
                    latest_loadcell_raw = raw;
                    atomic_store(&loadcell_data_valid, true);
                }
            }
            gpio_intr_enable(loadcell.drdy_pin);
        }
    }
}

void update_temp_state(void *arg)
{
    uint32_t gpio_num;
    
    while (1)
    {
        if (xQueueReceive(temp_ready_queue, &gpio_num, portMAX_DELAY))
        {
            if (xSemaphoreTake(spi_bus_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                float temp;
                esp_err_t err = max31865_get_temperature(&rtd, &temp);
                xSemaphoreGive(spi_bus_mutex);
                
                if (err == ESP_OK)
                {
                    latest_temp = temp;
                    atomic_store(&temp_data_valid, true);
                }
            }
            gpio_intr_enable(TEMP_DATA_READY);
        }
    }
}

void update_ina260_state(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1)
    {
        if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            int32_t v, i, p;
            esp_err_t err_v = ina260_get_voltage_mv(&power_monitor, &v);
            esp_err_t err_i = ina260_get_current_ma(&power_monitor, &i);
            esp_err_t err_p = ina260_get_power_mw(&power_monitor, &p);
            xSemaphoreGive(i2c_bus_mutex);
            
            if (err_v == ESP_OK && err_i == ESP_OK && err_p == ESP_OK)
            {
                latest_voltage_mv = v;
                latest_current_ma = i;
                latest_power_mw = p;
                atomic_store(&ina260_data_valid, true);
                // report_ina260_json(v, i, p);
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

void update_pd_state(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1)
    {
        if (xSemaphoreTake(i2c_bus_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            uint16_t voltage = pd_get_voltage_mv(&pd_device);
            xSemaphoreGive(i2c_bus_mutex);
            
            latest_pd_voltage_mv = voltage;
            atomic_store(&pd_data_valid, true);
            
            if (voltage > 19000) {
                led_set_color(&status_led, 255, 0, 255, 0);
            }
            else if (voltage > 6000) {   
                led_set_color(&status_led, 255, 255, 255, 0);
            }
            else {
                led_set_color(&status_led, 255, 255, 0, 0);
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
}

// =============================================================================
// PID Control Task
// =============================================================================
void pid_control_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    pid_last_time_us = esp_timer_get_time();
    
    while (1)
    {
        bool should_run = atomic_load(&heater_enabled);
        bool can_run = false;
        
        if (should_run) {
            if (latest_pd_voltage_mv < 12000) {
                can_run = false;
            }
            else if (!atomic_load(&temp_data_valid)) {
                can_run = false;
            }
            else if (latest_temp > 60.0f) {
                can_run = false;
            }
            else {
                can_run = true;
            }
        }
        
        atomic_store(&pid_active, can_run);
        
        if (can_run) {
            int64_t now_us = esp_timer_get_time();
            float dt = (now_us - pid_last_time_us) / 1000000.0f;
            pid_last_time_us = now_us;
            
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 1.0f) dt = 1.0f;
            
            float current_temp = latest_temp;
            float error = target_temperature - current_temp;
            
            float p_term = kp * error;
            
            pid_integral += error * dt;
            if (pid_integral > PID_INTEGRAL_MAX) pid_integral = PID_INTEGRAL_MAX;
            if (pid_integral < -PID_INTEGRAL_MAX) pid_integral = -PID_INTEGRAL_MAX;
            float i_term = ki * pid_integral;
            
            float derivative = (error - pid_prev_error) / dt;
            float d_term = kd * derivative;
            pid_prev_error = error;
            
            float output = p_term + i_term + d_term;
            
            if (output < 0.0f) output = 0.0f;
            if (output > (float)PID_MAX_DUTY) output = (float)PID_MAX_DUTY;
            
            heater_set_duty((uint32_t)output);
        }
        else {
            heater_off();
            if (!should_run) {
                pid_reset();
            }
        }
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PID_CONTROL_INTERVAL_MS));
    }
}

// =============================================================================
// Serial Command Processing
// =============================================================================
static void process_command_json(const char *cmd)
{
    cJSON *json = cJSON_Parse(cmd);
    if (json == NULL) {
        send_response("parse_error", false, "Invalid JSON");
        return;
    }
    
    cJSON *cmd_obj = cJSON_GetObjectItem(json, "cmd");
    if (cmd_obj == NULL || !cJSON_IsString(cmd_obj)) {
        send_response("unknown", false, "Missing 'cmd' field");
        cJSON_Delete(json);
        return;
    }
    
    const char *cmd_str = cmd_obj->valuestring;
    
    // -------------------------------------------------------------------------
    // heater_on
    // -------------------------------------------------------------------------
    if (strcmp(cmd_str, "heater_on") == 0) {
        pid_reset();
        atomic_store(&heater_enabled, true);
        send_response(cmd_str, true, "Heater enabled");
    }
    // -------------------------------------------------------------------------
    // heater_off
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "heater_off") == 0) {
        atomic_store(&heater_enabled, false);
        heater_off();
        send_response(cmd_str, true, "Heater disabled");
    }
    // -------------------------------------------------------------------------
    // gpio_test - Direct GPIO test (bypasses LEDC entirely)
    // Usage: {"cmd": "gpio_test", "state": 1, "duration_ms": 500}
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "gpio_test") == 0) {
        cJSON *state = cJSON_GetObjectItem(json, "state");
        cJSON *duration = cJSON_GetObjectItem(json, "duration_ms");
        
        int gpio_state = (state && cJSON_IsNumber(state)) ? (int)state->valuedouble : 1;
        int dur_ms = (duration && cJSON_IsNumber(duration)) ? (int)duration->valuedouble : 500;
        
        // Temporarily disable LEDC on this pin
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        
        // Configure as plain GPIO
        gpio_reset_pin(HEATER_PWM);
        gpio_set_direction(HEATER_PWM, GPIO_MODE_OUTPUT);
        gpio_set_level(HEATER_PWM, gpio_state);
        
        send_response(cmd_str, true, gpio_state ? "GPIO HIGH" : "GPIO LOW");
        
        if (dur_ms > 0 && gpio_state) {
            vTaskDelay(pdMS_TO_TICKS(dur_ms));
            gpio_set_level(HEATER_PWM, 0);
            
            // Re-init LEDC
            setupHeater();
            
            cJSON *resp = cJSON_CreateObject();
            cJSON_AddStringToObject(resp, "type", "gpio_test_done");
            cJSON_AddNumberToObject(resp, "duration_ms", dur_ms);
            print_json(resp);
        }
    }
    // -------------------------------------------------------------------------
    // pwm_test - Test LEDC PWM directly with specific duty
    // Usage: {"cmd": "pwm_test", "duty": 128}
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "pwm_test") == 0) {
        cJSON *duty_val = cJSON_GetObjectItem(json, "duty");
        uint32_t duty = (duty_val && cJSON_IsNumber(duty_val)) ? (uint32_t)duty_val->valuedouble : 128;
        
        if (duty > 255) duty = 255;
        
        esp_err_t err1 = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        esp_err_t err2 = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        
        // Also read back the duty to verify
        uint32_t read_duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        
        cJSON *resp = cJSON_CreateObject();
        cJSON_AddStringToObject(resp, "type", "response");
        cJSON_AddStringToObject(resp, "cmd", cmd_str);
        cJSON_AddBoolToObject(resp, "success", (err1 == ESP_OK && err2 == ESP_OK));
        cJSON_AddNumberToObject(resp, "duty_set", duty);
        cJSON_AddNumberToObject(resp, "duty_read", read_duty);
        cJSON_AddStringToObject(resp, "err1", esp_err_to_name(err1));
        cJSON_AddStringToObject(resp, "err2", esp_err_to_name(err2));
        cJSON_AddBoolToObject(resp, "ledc_init", ledc_initialized);
        print_json(resp);
    }
    // -------------------------------------------------------------------------
    // reinit_ledc - Reinitialize LEDC
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "reinit_ledc") == 0) {
        ledc_initialized = false;
        setupHeater();
        send_response(cmd_str, ledc_initialized, ledc_initialized ? "LEDC reinitialized" : "LEDC init failed");
    }
    // -------------------------------------------------------------------------
    // set_temp
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "set_temp") == 0) {
        cJSON *val = cJSON_GetObjectItem(json, "value");
        if (val && cJSON_IsNumber(val)) {
            float new_temp = (float)val->valuedouble;
            if (new_temp < PID_MIN_TARGET_TEMP) {
                send_response(cmd_str, false, "Temperature below minimum (20C)");
            }
            else if (new_temp > PID_MAX_TARGET_TEMP) {
                send_response(cmd_str, false, "Temperature above maximum (50C)");
            }
            else {
                target_temperature = new_temp;
                pid_integral = 0.0f;
                send_response(cmd_str, true, "Target temperature set");
            }
        }
        else {
            send_response(cmd_str, false, "Missing or invalid 'value'");
        }
    }
    // -------------------------------------------------------------------------
    // set_kp
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "set_kp") == 0) {
        cJSON *val = cJSON_GetObjectItem(json, "value");
        if (val && cJSON_IsNumber(val)) {
            float new_kp = (float)val->valuedouble;
            if (new_kp >= 0.0f) {
                kp = new_kp;
                send_response(cmd_str, true, "Kp set");
            }
            else {
                send_response(cmd_str, false, "Kp must be >= 0");
            }
        }
        else {
            send_response(cmd_str, false, "Missing or invalid 'value'");
        }
    }
    // -------------------------------------------------------------------------
    // set_ki
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "set_ki") == 0) {
        cJSON *val = cJSON_GetObjectItem(json, "value");
        if (val && cJSON_IsNumber(val)) {
            float new_ki = (float)val->valuedouble;
            if (new_ki >= 0.0f) {
                ki = new_ki;
                pid_integral = 0.0f;
                send_response(cmd_str, true, "Ki set");
            }
            else {
                send_response(cmd_str, false, "Ki must be >= 0");
            }
        }
        else {
            send_response(cmd_str, false, "Missing or invalid 'value'");
        }
    }
    // -------------------------------------------------------------------------
    // set_kd
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "set_kd") == 0) {
        cJSON *val = cJSON_GetObjectItem(json, "value");
        if (val && cJSON_IsNumber(val)) {
            float new_kd = (float)val->valuedouble;
            if (new_kd >= 0.0f) {
                kd = new_kd;
                send_response(cmd_str, true, "Kd set");
            }
            else {
                send_response(cmd_str, false, "Kd must be >= 0");
            }
        }
        else {
            send_response(cmd_str, false, "Missing or invalid 'value'");
        }
    }
    // -------------------------------------------------------------------------
    // set_pid
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "set_pid") == 0) {
        cJSON *val_kp = cJSON_GetObjectItem(json, "kp");
        cJSON *val_ki = cJSON_GetObjectItem(json, "ki");
        cJSON *val_kd = cJSON_GetObjectItem(json, "kd");
        
        bool valid = true;
        float new_kp = kp, new_ki = ki, new_kd = kd;
        
        if (val_kp && cJSON_IsNumber(val_kp)) {
            new_kp = (float)val_kp->valuedouble;
            if (new_kp < 0.0f) valid = false;
        }
        if (val_ki && cJSON_IsNumber(val_ki)) {
            new_ki = (float)val_ki->valuedouble;
            if (new_ki < 0.0f) valid = false;
        }
        if (val_kd && cJSON_IsNumber(val_kd)) {
            new_kd = (float)val_kd->valuedouble;
            if (new_kd < 0.0f) valid = false;
        }
        
        if (valid) {
            kp = new_kp;
            ki = new_ki;
            kd = new_kd;
            pid_integral = 0.0f;
            send_response(cmd_str, true, "PID gains set");
        }
        else {
            send_response(cmd_str, false, "Invalid PID values (must be >= 0)");
        }
    }
    // -------------------------------------------------------------------------
    // get_status
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "get_status") == 0) {
        report_pid_status_json();
    }
    // -------------------------------------------------------------------------
    // reset_pid
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "reset_pid") == 0) {
        pid_reset();
        send_response(cmd_str, true, "PID state reset");
    }
    // -------------------------------------------------------------------------
    // set_duty - Manual PWM duty
    // -------------------------------------------------------------------------
    else if (strcmp(cmd_str, "set_duty") == 0) {
        cJSON *val = cJSON_GetObjectItem(json, "value");
        if (val && cJSON_IsNumber(val)) {
            uint32_t duty = (uint32_t)val->valuedouble;
            if (duty > PID_MAX_DUTY) {
                duty = PID_MAX_DUTY;
            }
            heater_set_duty(duty);
            
            char msg[64];
            snprintf(msg, sizeof(msg), "Duty set to %lu (%.1f%%)", 
                     (unsigned long)duty, (duty * 100.0f) / 255.0f);
            send_response(cmd_str, true, msg);
        }
        else {
            send_response(cmd_str, false, "Missing or invalid 'value'");
        }
    }
    // -------------------------------------------------------------------------
    // Unknown
    // -------------------------------------------------------------------------
    else {
        send_response(cmd_str, false, "Unknown command");
    }
    
    cJSON_Delete(json);
}

static void usb_serial_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 256,
    };
    usb_serial_jtag_driver_install(&cfg);
}

static void command_task(void *arg)
{
    char buf[256];
    int idx = 0;
    
    while (1)
    {
        uint8_t byte;
        int len = usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(100));
        
        if (len > 0)
        {
            if (byte == '\n' || byte == '\r')
            {
                if (idx > 0)
                {
                    buf[idx] = '\0';
                    process_command_json(buf);
                    idx = 0;
                }
            }
            else if (idx < sizeof(buf) - 1)
            {
                buf[idx++] = byte;
            }
        }
    }
}

void status_report_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1)
    {
        report_pid_status_json();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    usb_serial_init();
    i2c_bus_mutex = xSemaphoreCreateMutex();
    spi_bus_mutex = xSemaphoreCreateMutex();
    json_print_mutex = xSemaphoreCreateMutex();
    
    led_init(&status_led);
    setupHeater();
    i2c_master_init();
    spi_init();
    
    pd_init(&pd_device);
    ina260_init(&power_monitor);
    max31865_init(&rtd);
    ads1220_init(&loadcell);
    
    gpio_config_t temp_io_conf = {
        .pin_bit_mask = (1ULL << TEMP_DATA_READY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,
    };
    gpio_config(&temp_io_conf);
    gpio_intr_disable(TEMP_DATA_READY);
    
    gpio_config_t loadcell_io_conf = {
        .pin_bit_mask = (1ULL << loadcell.drdy_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,
    };
    gpio_config(&loadcell_io_conf);
    gpio_intr_disable(loadcell.drdy_pin);
    
    float temp;
    max31865_get_temperature(&rtd, &temp);
    
    int32_t raw;
    ads1220_read_raw(&loadcell, &raw);
    
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    temp_ready_queue = xQueueCreate(10, sizeof(uint32_t));
    loadcell_ready_queue = xQueueCreate(10, sizeof(uint32_t));
    
    xTaskCreate(update_temp_state, "temp_update", 4096, NULL, 10, NULL);
    xTaskCreate(update_loadcell_state, "loadcell_update", 4096, NULL, 10, NULL);
    xTaskCreate(update_ina260_state, "ina260_update", 4096, NULL, 5, NULL);
    xTaskCreate(update_pd_state, "pd_update", 4096, NULL, 5, NULL);
    xTaskCreate(command_task, "command_task", 4096, NULL, 1, NULL);
    xTaskCreate(pid_control_task, "pid_control", 4096, NULL, 8, NULL);
    xTaskCreate(status_report_task, "status_report", 4096, NULL, 2, NULL);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TEMP_DATA_READY, temp_ready_isr_handler, (void*) TEMP_DATA_READY);
    gpio_isr_handler_add(loadcell.drdy_pin, loadcell_ready_isr_handler, (void*) loadcell.drdy_pin);
    
    gpio_intr_enable(TEMP_DATA_READY);
    gpio_intr_enable(loadcell.drdy_pin);
    
    ESP_LOGI(TAG, "Setup complete, LEDC initialized: %d", ledc_initialized);
}

void app_main(void)
{
    setup();
    
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
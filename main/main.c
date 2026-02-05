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
#include "nvs_flash.h"
#include "nvs.h"
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
#include "pid.h"
#include "board.h"
// #include "esp_littlefs.h"


static SemaphoreHandle_t i2c_bus_mutex = NULL;
static SemaphoreHandle_t spi_bus_mutex = NULL;
atomic_int acnt;

led_ctrl_t status_led = {
    .clk_pin = LED_CLOCK,
    .data_pin = LED_DATA_IN,
};

ina260_t power_monitor = {
    .i2c_port = 0,
    .i2c_addr = 0x40,
    .alert_pin = 15, 
};

ads1220_t loadcell = {
    .spi_host = SPI2_HOST,
    .cs_pin = 17,
    .drdy_pin = 16,
    .clock_hz = 5000000,
};

max31865_t rtd = {
    .spi_host = SPI2_HOST,
    .cs_pin = 20,
    .rref = 430.0f,
    .rtd_nominal = 100.0f,
    .three_wire = false,
    .clock_hz = 5000000,
};

pd_t pd_device = {
    .i2c_port = 0,
    .i2c_addr = PD_ADDRESS,
};

pid_controller_t pid_c = {
    .kp = 5.0f,
    .ki = 0.1f,
    .kd = 2.0f,
    .pwm_pin = HEATER_PWM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,     
    .ledc_channel = LEDC_CHANNEL_0,
    .ledc_clk_cfg = LEDC_AUTO_CLK,
    .frequency = 100,
    .max_duty = 255,
    .enabled = false,
};


// Temperature state
static volatile float current_temperature = 0.0f;
static volatile float target_temperature = 0.0f;
static atomic_bool temp_data_valid = false;
static QueueHandle_t temp_ready_queue = NULL;

// Loadcell state
static volatile int32_t latest_loadcell_raw = 0;
static volatile int32_t loadcell_zero = 0;
static volatile float  loadcell_multiplier = 1.0f;
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

esp_err_t nvs_write_integer(const char *key, int value) {
   nvs_handle_t handle;
   esp_err_t err;
   err = nvs_open("storage", NVS_READWRITE, &handle);
   if (err != ESP_OK) {
       printf("Error opening NVS: %s", esp_err_to_name(err));
       return err;
   }
   err = nvs_set_blob(handle, key, &value, sizeof(int));
   if (err != ESP_OK) {
       printf("Error writing float: %s", esp_err_to_name(err));
       nvs_close(handle);
       return err;
   }
   err = nvs_commit(handle);
   if (err != ESP_OK) {
       printf("Error committing: %s", esp_err_to_name(err));
   }
   nvs_close(handle);
   return err;
}

int nvs_read_integer(const char *key, int default_value) {
   nvs_handle_t handle;
   esp_err_t err;
   float value = default_value;
   err = nvs_open("storage", NVS_READONLY, &handle);
   if (err != ESP_OK) {
       printf("Error opening NVS: %s", esp_err_to_name(err));
       return default_value;
   }
   size_t required_size = sizeof(int);
   err = nvs_get_blob(handle, key, &value, &required_size);
   if (err == ESP_ERR_NVS_NOT_FOUND) {
       printf( "Key '%s' not found, using default", key);
   } else if (err != ESP_OK) {
       printf( "Error reading float: %s", esp_err_to_name(err));
   }
   nvs_close(handle);
   return value;
}

esp_err_t nvs_write_float(const char *key, float value) {
   nvs_handle_t handle;
   esp_err_t err;
   err = nvs_open("storage", NVS_READWRITE, &handle);
   if (err != ESP_OK) {
       printf("Error opening NVS: %s", esp_err_to_name(err));
       return err;
   }
   err = nvs_set_blob(handle, key, &value, sizeof(float));
   if (err != ESP_OK) {
       printf("Error writing float: %s", esp_err_to_name(err));
       nvs_close(handle);
       return err;
   }
   err = nvs_commit(handle);
   if (err != ESP_OK) {
       printf("Error committing: %s", esp_err_to_name(err));
   }
   nvs_close(handle);
   return err;
}

float nvs_read_float(const char *key, float default_value) {
   nvs_handle_t handle;
   esp_err_t err;
   float value = default_value;
   err = nvs_open("storage", NVS_READONLY, &handle);
   if (err != ESP_OK) {
       printf("Error opening NVS: %s", esp_err_to_name(err));
       return default_value;
   }
   size_t required_size = sizeof(float);
   err = nvs_get_blob(handle, key, &value, &required_size);
   if (err == ESP_ERR_NVS_NOT_FOUND) {
       printf( "Key '%s' not found, using default", key);
   } else if (err != ESP_OK) {
       printf( "Error reading float: %s", esp_err_to_name(err));
   }
   nvs_close(handle);
   return value;
}

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

static void report_error_json(const char *message)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "type", "error");
    cJSON_AddStringToObject(json, "message", message);;
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

static void process_command_json(const char *cmd)
{
    cJSON *json = cJSON_Parse(cmd);
    if (json == NULL) {
        send_response("parse_error", false, "Invalid JSON");
        return;
    }
    
    // Command list
    // { "type": "set_target_temperature", "value": float }
    // { "type": "enable_heater", "value": bool }
    
    cJSON *type_item = cJSON_GetObjectItem(json, "type");
    if (type_item == NULL || type_item->type != cJSON_String) {
        send_response("invalid_command", false, "Missing or invalid 'type' field");
        cJSON_Delete(json);
        return;
    }
    
    const char *type_str = type_item->valuestring;
    
    if (strcmp(type_str, "set_target_temperature") == 0) {
        cJSON *value_item = cJSON_GetObjectItem(json, "value");
        if (value_item != NULL && value_item->type == cJSON_Number) {
            target_temperature = (float)value_item->valuedouble;
            send_response("set_target_temperature", true, "Target temperature set");
        } else {
            send_response("set_target_temperature", false, "Invalid 'value' field");
        }
    }
    
    else if (strcmp(type_str, "set_loadcell_zero") == 0) {
        loadcell_zero = latest_loadcell_raw;
        nvs_write_integer("zero", loadcell_zero);
    }

     else if (strcmp(type_str, "set_loadcell_multiplier") == 0) {
        cJSON *value_item = cJSON_GetObjectItem(json, "value");
        if (value_item != NULL && value_item->type == cJSON_Number) {
            float current_force_in_grams = (float)value_item->valuedouble;
            loadcell_multiplier = current_force_in_grams / (latest_loadcell_raw - loadcell_zero);
            nvs_write_float("multiplier", loadcell_multiplier);
            send_response("set_loadcell_multiplier", true, "Force set");
        } else {
            send_response("set_loadcell_multiplier", false, "Invalid 'value' field");
        }
    }
    
    else if (strcmp(type_str, "enable_heater") == 0) {
        cJSON *value_item = cJSON_GetObjectItem(json, "value");
        if (value_item != NULL && value_item->type == cJSON_True) {
            // Check if we have valid temperature data before enabling
            if (atomic_load(&temp_data_valid)) {
                pid_set_enabled(&pid_c, true);
                send_response("enable_heater", true, "Heater enabled");
            } else {
                report_error_json("No valid temperature data");
            }
        } else if (value_item != NULL && value_item->type == cJSON_False) {
            pid_set_enabled(&pid_c, false);
            send_response("enable_heater", true, "Heater disabled");
        } else {
            send_response("enable_heater", false, "Invalid 'value' field");
        }
    } else {
        report_error_json("Unknown command type");
    }
    
    cJSON_Delete(json);
}

// =============================SetupW Functions =================================

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

static void usb_serial_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 256,
    };
    usb_serial_jtag_driver_install(&cfg);
}

// =============================ISR handlers =================================

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

// =============================Task Functions=============================
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
                    current_temperature = temp;
                    atomic_store(&temp_data_valid, true);
                    print_json(report_temperature_json(temp));
                    // if the temperature exceeds target by more than 20c or is out of range disable the heater
                    if ((pid_c.enabled) && (temp > target_temperature + 20.0f || temp < -200.0f || temp > 200.0f)) {
                        pid_set_enabled(&pid_c, false);
                        report_error_json("Temperature out of range - heater disabled");
                    }
                    int ret = pid_update(&pid_c, target_temperature, current_temperature);
                    cJSON *json = cJSON_CreateObject();
                    cJSON_AddStringToObject(json, "type", "pid_status");
                    cJSON_AddNumberToObject(json, "target_temperature", target_temperature);
                    cJSON_AddBoolToObject(json, "heater_enabled", pid_c.enabled);
                    cJSON_AddNumberToObject(json, "pwm_duty", ret);
                    print_json(json);
                }
            }
            gpio_intr_enable(TEMP_DATA_READY);
        }
    }
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
                    print_json(report_loadcell_json(raw, loadcell_zero, loadcell_multiplier));
                }
            }
            gpio_intr_enable(loadcell.drdy_pin);
        }
    }
}

static void update_ina260_state(void *arg)
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
                // if the voltage is below 15000 mV we disable the heater
                if (pid_c.enabled && v < 15000) {
                    pid_set_enabled(&pid_c, false);
                    report_error_json("INA260 voltage low - heater disabled");
                }
                print_json(report_ina260_json(v, i, p));
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(100));
    }
}

static void update_pd_state(void *arg)
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
            print_json(report_pd_json(voltage));
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(500));
    }
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




static const char *TAG = "main";

void setup()
{
    
    pid_init(&pid_c);
    usb_serial_init();
    i2c_bus_mutex = xSemaphoreCreateMutex();
    spi_bus_mutex = xSemaphoreCreateMutex();
    json_print_mutex = xSemaphoreCreateMutex();
    
    led_init(&status_led);
    i2c_master_init();
    spi_init();

    loadcell_zero = nvs_read_integer("zero", 0);
    loadcell_multiplier = nvs_read_float("multiplier", 1.0);
    
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
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(TEMP_DATA_READY, temp_ready_isr_handler, (void*) TEMP_DATA_READY);
    gpio_isr_handler_add(loadcell.drdy_pin, loadcell_ready_isr_handler, (void*) loadcell.drdy_pin);
    
    gpio_intr_enable(TEMP_DATA_READY);
    gpio_intr_enable(loadcell.drdy_pin);
    


    // Init NVS
    esp_err_t err;
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    nvs_handle_t my_handle;

}

void app_main(void)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    setup();
    
    while (1)
    {   
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
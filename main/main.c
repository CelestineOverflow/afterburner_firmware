#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>
#include "driver/spi_master.h"

// APA102 Like LED 
#define LED_CLOCK 4
#define LED_DATA_IN 5
#define NUM_LEDS 1

// Power Delivery TPS25730
#define PD_POWER_ON 6
#define PD_ADDRESS 0x20

// TPS25730 Register Addresses
#define TPS25730_REG_ACTIVE_CONTRACT_PDO  0x34  // Length 6
#define TPS25730_REG_ACTIVE_CONTRACT_RDO  0x35  // Length 4

// Heater Mosfet PWM
#define HEATER_PWM 7

// Power Monitor INA260AIPWR
#define POWER_MONITOR_ADDRESS 0x40
#define ALERT_INT 15
#define INA260_REG_CONFIG      0x00
#define INA260_REG_CURRENT     0x01  // 1.25 mA/LSB
#define INA260_REG_VOLTAGE     0x02  // 1.25 mV/LSB
#define INA260_REG_POWER       0x03  // 10 mW/LSB
#define INA260_REG_MASK_EN     0x06
#define INA260_REG_ALERT       0x07
#define INA260_REG_MFG_ID      0xFE
#define INA260_REG_DIE_ID      0xFF

// I2C Pins
#define SDA 18
#define SCL 19
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 100

// SPI
#define MOSI 22
#define MISO 23
#define SCK 21

//MAX31865
#define TEMP_CS 20
#define TEMP_DATA_READY 3

// MAX31865 Register Addresses (read: addr, write: addr | 0x80)
#define MAX31865_REG_CONFIG      0x00
#define MAX31865_REG_RTD_MSB     0x01
#define MAX31865_REG_RTD_LSB     0x02
#define MAX31865_REG_HFAULT_MSB  0x03
#define MAX31865_REG_HFAULT_LSB  0x04
#define MAX31865_REG_LFAULT_MSB  0x05
#define MAX31865_REG_LFAULT_LSB  0x06
#define MAX31865_REG_FAULT       0x07

// Configuration bits
#define MAX31865_CONFIG_VBIAS    0x80
#define MAX31865_CONFIG_AUTO     0x40
#define MAX31865_CONFIG_1SHOT    0x20
#define MAX31865_CONFIG_3WIRE    0x10
#define MAX31865_CONFIG_FAULT_CLR 0x02
#define MAX31865_CONFIG_50HZ     0x01  // 0 = 60Hz, 1 = 50Hz

// RTD Configuration - adjust these for your setup!
#define RTD_RREF         430.0f    // Reference resistor (430 for PT100, 4300 for PT1000)
#define RTD_NOMINAL      100.0f    // RTD nominal resistance at 0°C (100 for PT100, 1000 for PT1000)

// Callendar-Van Dusen coefficients for PT100
#define RTD_A  3.9083e-3f
#define RTD_B -5.775e-7f



static spi_device_handle_t max31865_spi = NULL;


static const char *TAG = "main";

// ============== ADS1220 24-bit ADC (Load Cell) ==============

// ADS1220 Commands
#define ADS1220_CMD_RESET     0x06
#define ADS1220_CMD_START     0x08
#define ADS1220_CMD_POWERDOWN 0x02
#define ADS1220_CMD_RDATA     0x10
#define ADS1220_CMD_RREG      0x20
#define ADS1220_CMD_WREG      0x40

// ADS1220 Register Addresses
#define ADS1220_REG0          0x00
#define ADS1220_REG1          0x01
#define ADS1220_REG2          0x02
#define ADS1220_REG3          0x03

// Register 0: Input MUX, Gain, PGA bypass
#define ADS1220_MUX_AIN0_AIN1   (0x00 << 4)  // Differential AIN0-AIN1
#define ADS1220_MUX_AIN2_AIN3   (0x03 << 4)  // Differential AIN2-AIN3
#define ADS1220_MUX_AIN0_AVSS   (0x08 << 4)  // Single-ended AIN0
#define ADS1220_MUX_AIN1_AVSS   (0x09 << 4)  // Single-ended AIN1
#define ADS1220_GAIN_1          (0x00 << 1)
#define ADS1220_GAIN_2          (0x01 << 1)
#define ADS1220_GAIN_4          (0x02 << 1)
#define ADS1220_GAIN_8          (0x03 << 1)
#define ADS1220_GAIN_16         (0x04 << 1)
#define ADS1220_GAIN_32         (0x05 << 1)
#define ADS1220_GAIN_64         (0x06 << 1)
#define ADS1220_GAIN_128        (0x07 << 1)
#define ADS1220_PGA_BYPASS      0x01

// Register 1: Data rate, Operating mode, Conversion mode, Temp sensor
#define ADS1220_DR_20SPS        (0x00 << 5)
#define ADS1220_DR_45SPS        (0x01 << 5)
#define ADS1220_DR_90SPS        (0x02 << 5)
#define ADS1220_DR_175SPS       (0x03 << 5)
#define ADS1220_DR_330SPS       (0x04 << 5)
#define ADS1220_DR_600SPS       (0x05 << 5)
#define ADS1220_DR_1000SPS      (0x06 << 5)
#define ADS1220_MODE_NORMAL     (0x00 << 3)
#define ADS1220_MODE_DUTY       (0x01 << 3)
#define ADS1220_MODE_TURBO      (0x02 << 3)
#define ADS1220_CONV_SINGLE     (0x00 << 2)
#define ADS1220_CONV_CONTINUOUS (0x01 << 2)
#define ADS1220_TEMP_DISABLE    (0x00 << 1)
#define ADS1220_TEMP_ENABLE     (0x01 << 1)

// Register 2: Voltage reference, FIR filter, Power switch, IDAC
#define ADS1220_VREF_INTERNAL   (0x00 << 6)  // 2.048V internal
#define ADS1220_VREF_EXT_AIN    (0x01 << 6)  // External on AIN0/AIN3
#define ADS1220_VREF_AVDD       (0x03 << 6)  // Analog supply
#define ADS1220_FIR_NONE        (0x00 << 4)
#define ADS1220_FIR_50_60       (0x01 << 4)
#define ADS1220_FIR_50          (0x02 << 4)
#define ADS1220_FIR_60          (0x03 << 4)
#define ADS1220_PSW_OPEN        (0x00 << 3)
#define ADS1220_PSW_AUTO        (0x01 << 3)
#define ADS1220_IDAC_OFF        0x00
#define ADS1220_IDAC_50UA       0x02
#define ADS1220_IDAC_100UA      0x03
#define ADS1220_IDAC_250UA      0x04
#define ADS1220_IDAC_500UA      0x05
#define ADS1220_IDAC_1000UA     0x06
#define ADS1220_IDAC_1500UA     0x07

// Register 3: IDAC routing, DRDY mode
#define ADS1220_IDAC1_DISABLED  (0x00 << 5)
#define ADS1220_IDAC1_AIN0      (0x01 << 5)
#define ADS1220_IDAC1_AIN1      (0x02 << 5)
#define ADS1220_IDAC2_DISABLED  (0x00 << 2)
#define ADS1220_DRDY_ONLY       (0x00 << 1)
//ADS1220IRVAR
#define LOADCELL_DRDY 16
#define LOADCELL_CS 17
static spi_device_handle_t ads1220_spi = NULL;

// Send command
static esp_err_t ads1220_send_cmd(uint8_t cmd) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    return spi_device_transmit(ads1220_spi, &t);
}

// Write register
static esp_err_t ads1220_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { ADS1220_CMD_WREG | (reg << 2), value };
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    
    return spi_device_transmit(ads1220_spi, &t);
}

// Read register
static esp_err_t ads1220_read_reg(uint8_t reg, uint8_t *value) {
    uint8_t tx[2] = { ADS1220_CMD_RREG | (reg << 2), 0x00 };
    uint8_t rx[2] = {0};
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    
    esp_err_t ret = spi_device_transmit(ads1220_spi, &t);
    if (ret == ESP_OK) {
        *value = rx[1];
    }
    return ret;
}

// Check if data is ready (DRDY pin low)
static bool ads1220_data_ready(void) {
    return gpio_get_level(LOADCELL_DRDY) == 0;
}

// Wait for data ready with timeout
static bool ads1220_wait_drdy(uint32_t timeout_ms) {
    uint32_t start = xTaskGetTickCount();
    while (!ads1220_data_ready()) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

// Read 24-bit conversion data
static esp_err_t ads1220_read_data(int32_t *data) {
    uint8_t tx[4] = { ADS1220_CMD_RDATA, 0x00, 0x00, 0x00 };
    uint8_t rx[4] = {0};
    
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    
    esp_err_t ret = spi_device_transmit(ads1220_spi, &t);
    if (ret == ESP_OK) {
        // 24-bit signed value, MSB first
        int32_t raw = ((int32_t)rx[1] << 16) | ((int32_t)rx[2] << 8) | rx[3];
        
        // Sign extend from 24-bit to 32-bit
        if (raw & 0x800000) {
            raw |= 0xFF000000;
        }
        
        *data = raw;
    }
    return ret;
}

// Start single conversion
static esp_err_t ads1220_start_conversion(void) {
    return ads1220_send_cmd(ADS1220_CMD_START);
}

// Initialize ADS1220
static esp_err_t ads1220_init(void) {
    esp_err_t ret;
    
    // Configure DRDY pin as input
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LOADCELL_DRDY),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // SPI device config (SPI bus already initialized by MAX31865)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1 MHz
        .mode = 1,                   // SPI mode 1
        .spics_io_num = LOADCELL_CS,
        .queue_size = 1,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &ads1220_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADS1220 SPI add device failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Reset the device
    ads1220_send_cmd(ADS1220_CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure for load cell:
    // Reg0: Differential AIN0-AIN1, Gain=128
    ret = ads1220_write_reg(ADS1220_REG0, ADS1220_MUX_AIN0_AIN1 | ADS1220_GAIN_128);
    if (ret != ESP_OK) return ret;
    
    // Reg1: 20 SPS (best noise), Normal mode, Continuous conversion
    ret = ads1220_write_reg(ADS1220_REG1, ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CONV_CONTINUOUS);
    if (ret != ESP_OK) return ret;
    
    // Reg2: Internal 2.048V reference, 50/60Hz rejection
    ret = ads1220_write_reg(ADS1220_REG2, ADS1220_VREF_INTERNAL | ADS1220_FIR_50_60 | ADS1220_IDAC_OFF);
    if (ret != ESP_OK) return ret;
    
    // Reg3: IDAC disabled, DRDY only on DOUT
    ret = ads1220_write_reg(ADS1220_REG3, ADS1220_IDAC1_DISABLED | ADS1220_IDAC2_DISABLED | ADS1220_DRDY_ONLY);
    if (ret != ESP_OK) return ret;
    
    // Start continuous conversion
    ads1220_start_conversion();
    
    ESP_LOGI(TAG, "ADS1220 initialized");
    return ESP_OK;
}

// Convert raw value to voltage (with internal 2.048V ref and gain)
static float ads1220_raw_to_voltage(int32_t raw, uint8_t gain) {
    // Full scale = 2.048V / gain
    // Resolution = 2^23 = 8388608
    float vref = 2.048f;
    return ((float)raw * vref) / (8388608.0f * (float)gain);
}

// Get raw ADC reading
static int32_t ads1220_get_raw(void) {
    int32_t data = 0;
    
    if (!ads1220_wait_drdy(100)) {
        ESP_LOGW(TAG, "ADS1220 DRDY timeout");
        return 0;
    }
    
    if (ads1220_read_data(&data) != ESP_OK) {
        return 0;
    }
    
    return data;
}

// Print ADS1220 status
static void ads1220_print_status(void) {
    int32_t raw;
    float voltage;
    
    // ESP_LOGI(TAG, "=== ADS1220 Load Cell ADC ===");
    
    if (!ads1220_wait_drdy(100)) {
        ESP_LOGE(TAG, "DRDY timeout - no data");
        return;
    }
    
    if (ads1220_read_data(&raw) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data");
        return;
    }
    
    voltage = ads1220_raw_to_voltage(raw, 128);  // Using gain=128
    
    // ESP_LOGI(TAG, "Raw: %ld (0x%06lX)", raw, raw & 0xFFFFFF);
    ESP_LOGI(TAG, "Voltage: %.6f V (%.3f mV)", voltage, voltage * 1000.0f);
    
    // For load cell, you'd convert voltage to weight here
    // Weight = (voltage / excitation / sensitivity) * full_scale
    // Example: 2mV/V sensitivity, 5V excitation, 10kg cell
    // float weight_kg = (voltage / (0.002f * 5.0f)) * 10.0f;
    
    // ESP_LOGI(TAG, "=============================");
}



// Write single register
static esp_err_t max31865_write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg | 0x80, value };  // Set bit 7 for write
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    
    return spi_device_transmit(max31865_spi, &t);
}
// Initialize SPI bus and MAX31865
static esp_err_t max31865_init(void) {
    esp_err_t ret;
    
    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .mosi_io_num = MOSI,
        .miso_io_num = MISO,
        .sclk_io_num = SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    
    // Initialize SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {  // Already initialized is OK
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // MAX31865 device configuration
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1 MHz (MAX31865 supports up to 5MHz)
        .mode = 1,                   // SPI mode 1 (CPOL=0, CPHA=1) or mode 3
        .spics_io_num = TEMP_CS,
        .queue_size = 1,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &max31865_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure MAX31865: VBIAS on, auto conversion, 50Hz filter (for Europe)
    uint8_t config = MAX31865_CONFIG_VBIAS | MAX31865_CONFIG_AUTO | MAX31865_CONFIG_50HZ;
    // Add MAX31865_CONFIG_3WIRE if using 3-wire RTD
    
    ret = max31865_write_reg(MAX31865_REG_CONFIG, config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX31865 config failed");
        return ret;
    }
    
    // Small delay for bias voltage to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "MAX31865 initialized");
    return ESP_OK;
}



// Read single register
static esp_err_t max31865_read_reg(uint8_t reg, uint8_t *value) {
    uint8_t tx[2] = { reg & 0x7F, 0x00 };  // Clear bit 7 for read
    uint8_t rx[2] = {0};
    
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    
    esp_err_t ret = spi_device_transmit(max31865_spi, &t);
    if (ret == ESP_OK) {
        *value = rx[1];
    }
    return ret;
}

// Read 16-bit RTD value
static esp_err_t max31865_read_rtd(uint16_t *rtd_raw) {
    uint8_t tx[3] = { MAX31865_REG_RTD_MSB & 0x7F, 0x00, 0x00 };
    uint8_t rx[3] = {0};
    
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    
    esp_err_t ret = spi_device_transmit(max31865_spi, &t);
    if (ret == ESP_OK) {
        // Combine MSB and LSB, remove fault bit (bit 0)
        *rtd_raw = ((rx[1] << 8) | rx[2]) >> 1;
    }
    return ret;
}

// Read fault status
static uint8_t max31865_read_fault(void) {
    uint8_t fault = 0;
    max31865_read_reg(MAX31865_REG_FAULT, &fault);
    return fault;
}

// Clear fault status
static void max31865_clear_fault(void) {
    uint8_t config;
    max31865_read_reg(MAX31865_REG_CONFIG, &config);
    config |= MAX31865_CONFIG_FAULT_CLR;
    max31865_write_reg(MAX31865_REG_CONFIG, config);
}

// Convert RTD raw value to resistance
static float max31865_rtd_to_resistance(uint16_t rtd_raw) {
    return (float)rtd_raw * RTD_RREF / 32768.0f;
}

// Convert resistance to temperature using Callendar-Van Dusen equation
// Simplified for positive temperatures (more accurate above 0°C)
static float max31865_resistance_to_temp(float resistance) {
    // For temperatures > 0°C:
    // T = (-A + sqrt(A² - 4B(1 - R/R0))) / (2B)
    
    float Z1 = -RTD_A;
    float Z2 = RTD_A * RTD_A - (4.0f * RTD_B);
    float Z3 = (4.0f * RTD_B) / RTD_NOMINAL;
    float Z4 = 2.0f * RTD_B;
    
    float temp = Z2 + (Z3 * resistance);
    temp = (sqrtf(temp) + Z1) / Z4;
    
    // For more accurate negative temperatures, a different formula is needed
    // or use a lookup table
    
    return temp;
}

// Get temperature in Celsius
static float max31865_get_temperature(void) {
    uint16_t rtd_raw;
    
    if (max31865_read_rtd(&rtd_raw) != ESP_OK) {
        return -999.0f;  // Error value
    }
    
    float resistance = max31865_rtd_to_resistance(rtd_raw);
    float temp = max31865_resistance_to_temp(resistance);
    
    return temp;
}

// Print MAX31865 status
static void max31865_print_status(void) {
    uint16_t rtd_raw;
    uint8_t fault;
    
    ESP_LOGI(TAG, "=== MAX31865 RTD Sensor ===");
    
    if (max31865_read_rtd(&rtd_raw) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RTD");
        return;
    }
    
    float resistance = max31865_rtd_to_resistance(rtd_raw);
    float temp = max31865_resistance_to_temp(resistance);
    
    ESP_LOGI(TAG, "RTD Raw: %u", rtd_raw);
    ESP_LOGI(TAG, "Resistance: %.2f Ohm", resistance);
    ESP_LOGI(TAG, "Temperature: %.2f °C", temp);
    
    fault = max31865_read_fault();
    if (fault) {
        ESP_LOGW(TAG, "Fault detected: 0x%02X", fault);
        if (fault & 0x80) ESP_LOGW(TAG, "  - RTD High Threshold");
        if (fault & 0x40) ESP_LOGW(TAG, "  - RTD Low Threshold");
        if (fault & 0x20) ESP_LOGW(TAG, "  - REFIN- > 0.85 x VBIAS");
        if (fault & 0x10) ESP_LOGW(TAG, "  - REFIN- < 0.85 x VBIAS (FORCE- open)");
        if (fault & 0x08) ESP_LOGW(TAG, "  - RTDIN- < 0.85 x VBIAS (FORCE- open)");
        if (fault & 0x04) ESP_LOGW(TAG, "  - Overvoltage/undervoltage");
        max31865_clear_fault();
    }
    
    ESP_LOGI(TAG, "===========================");
}

// Read 16-bit register from INA260
static esp_err_t ina260_read_reg(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    esp_err_t ret;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (POWER_MONITOR_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (POWER_MONITOR_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        *value = (data[0] << 8) | data[1];  // Big-endian
    }
    
    return ret;
}

// Get bus voltage in millivolts
static int32_t ina260_get_voltage_mv(void) {
    uint16_t raw;
    if (ina260_read_reg(INA260_REG_VOLTAGE, &raw) == ESP_OK) {
        // 1.25 mV/LSB
        return (int32_t)raw * 125 / 100;
    }
    return -1;
}

// Get current in milliamps (signed, can be negative)
static int32_t ina260_get_current_ma(void) {
    uint16_t raw;
    if (ina260_read_reg(INA260_REG_CURRENT, &raw) == ESP_OK) {
        // 1.25 mA/LSB, signed value
        int16_t signed_raw = (int16_t)raw;
        return (int32_t)signed_raw * 125 / 100;
    }
    return 0;
}

// Get power in milliwatts
static int32_t ina260_get_power_mw(void) {
    uint16_t raw;
    if (ina260_read_reg(INA260_REG_POWER, &raw) == ESP_OK) {
        // 10 mW/LSB
        return (int32_t)raw * 10;
    }
    return -1;
}

// Print all INA260 readings
static void ina260_print_status(void) {
    int32_t voltage_mv = ina260_get_voltage_mv();
    int32_t current_ma = ina260_get_current_ma();
    int32_t power_mw = ina260_get_power_mw();
    
    ESP_LOGI(TAG, "=== INA260 Power Monitor ===");
    
    if (voltage_mv >= 0) {
        ESP_LOGI(TAG, "Voltage: %ld mV (%.2fV)", voltage_mv, voltage_mv / 1000.0f);
    } else {
        ESP_LOGE(TAG, "Voltage: READ ERROR");
    }
    
    ESP_LOGI(TAG, "Current: %ld mA (%.3fA)", current_ma, current_ma / 1000.0f);
    
    if (power_mw >= 0) {
        ESP_LOGI(TAG, "Power: %ld mW (%.2fW)", power_mw, power_mw / 1000.0f);
    } else {
        ESP_LOGE(TAG, "Power: READ ERROR");
    }
    
    ESP_LOGI(TAG, "============================");
}






// PD Status structure
typedef struct {
    bool contract_valid;
    bool capability_mismatch;
    uint8_t object_position;
    uint16_t voltage_mv;
    uint16_t current_ma;
    uint8_t supply_type;
} pd_status_t;

static void led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_CLOCK) | (1ULL << LED_DATA_IN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_CLOCK, 0);
    gpio_set_level(LED_DATA_IN, 0);
}

static void led_send_byte(uint8_t byte) {
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(LED_DATA_IN, (byte >> i) & 1);
        gpio_set_level(LED_CLOCK, 1);
        gpio_set_level(LED_CLOCK, 0);
    }
}

static void led_set_color(uint8_t brightness, uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < 4; i++) {
        led_send_byte(0x00);
    }
    led_send_byte(0xE0 | (brightness & 0x1F));
    led_send_byte(b);
    led_send_byte(g);
    led_send_byte(r);
    for (int i = 0; i < 4; i++) {
        led_send_byte(0xFF);
    }
}

void setupHeater(){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << HEATER_PWM),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(HEATER_PWM, 0);
}

static void i2c_master_init(void) {
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

static bool i2c_check_device(uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK;
}

static void i2c_scan(void) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    uint8_t count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_check_device(addr)) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            count++;
        }
    }
    ESP_LOGI(TAG, "I2C scan complete. Found %d device(s)", count);
}

// Read TPS25730 register with retry logic
static esp_err_t tps25730_read_register(uint8_t reg, uint8_t *data, uint8_t expected_len) {
    uint8_t buffer[16];
    esp_err_t ret;
    uint8_t total = expected_len + 1;  // +1 for byte count
    
    // Try up to 3 times
    for (int attempt = 0; attempt < 3; attempt++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PD_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PD_ADDRESS << 1) | I2C_MASTER_READ, true);
        
        for (int i = 0; i < total - 1; i++) {
            i2c_master_read_byte(cmd, &buffer[i], I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, &buffer[total - 1], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(200));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            // Verify byte count matches expected
            if (buffer[0] == expected_len || buffer[0] == 0) {
                memcpy(data, &buffer[1], expected_len);
                return ESP_OK;
            }
        }
        
        // Small delay before retry
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ret;
}

// Read PD status with full parsing
static esp_err_t pd_read_status(pd_status_t *status) {
    uint8_t pdo_data[6] = {0};
    uint8_t rdo_data[4] = {0};
    esp_err_t ret;
    
    memset(status, 0, sizeof(pd_status_t));
    
    // Read ACTIVE_CONTRACT_PDO (6 bytes)
    ret = tps25730_read_register(TPS25730_REG_ACTIVE_CONTRACT_PDO, pdo_data, 6);
    
    if (ret != ESP_OK) {
        // No contract or read failed
        status->contract_valid = false;
        status->voltage_mv = 5000;  // Default 5V
        return ret;
    }
    
    // Parse PDO (bytes 0-3, little-endian)
    uint32_t pdo = pdo_data[0] | (pdo_data[1] << 8) | (pdo_data[2] << 16) | (pdo_data[3] << 24);
    
    if (pdo == 0) {
        status->contract_valid = false;
        status->voltage_mv = 5000;
        return ESP_OK;
    }
    
    status->supply_type = (pdo >> 30) & 0x03;
    
    switch (status->supply_type) {
        case 0:  // Fixed Supply
            status->voltage_mv = ((pdo >> 10) & 0x3FF) * 50;
            status->current_ma = (pdo & 0x3FF) * 10;
            break;
        case 1:  // Variable Supply
        case 2:  // Battery
            status->voltage_mv = ((pdo >> 20) & 0x3FF) * 50;  // Max voltage
            status->current_ma = (pdo & 0x3FF) * 10;
            break;
        case 3:  // APDO (PPS)
            status->voltage_mv = ((pdo >> 17) & 0xFF) * 100;  // Max voltage
            status->current_ma = (pdo & 0x7F) * 50;
            break;
    }
    
    status->contract_valid = (status->voltage_mv > 0);
    
    // Read ACTIVE_CONTRACT_RDO (4 bytes)
    ret = tps25730_read_register(TPS25730_REG_ACTIVE_CONTRACT_RDO, rdo_data, 4);
    
    if (ret == ESP_OK) {
        uint32_t rdo = rdo_data[0] | (rdo_data[1] << 8) | (rdo_data[2] << 16) | (rdo_data[3] << 24);
        
        if (rdo != 0) {
            status->object_position = (rdo >> 28) & 0x07;
            status->capability_mismatch = (rdo >> 26) & 0x01;
            // RDO current overrides PDO current (actual negotiated)
            uint16_t rdo_current = ((rdo >> 10) & 0x3FF) * 10;
            if (rdo_current > 0) {
                status->current_ma = rdo_current;
            }
        }
    }
    
    return ESP_OK;
}

// Get voltage in millivolts
static uint16_t pd_get_voltage_mv(void) {
    pd_status_t status;
    if (pd_read_status(&status) == ESP_OK) {
        return status.voltage_mv;
    }
    return 5000;  // Default
}

// Check if PD contract is active
static bool pd_is_contract_active(void) {
    pd_status_t status;
    if (pd_read_status(&status) == ESP_OK) {
        return status.contract_valid;
    }
    return false;
}

// Print PD status
static void pd_print_status(void) {
    pd_status_t status;
    
    if (pd_read_status(&status) != ESP_OK) {
        ESP_LOGW(TAG, "PD read failed, retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
        if (pd_read_status(&status) != ESP_OK) {
            ESP_LOGE(TAG, "PD status unavailable");
            return;
        }
    }
    
    ESP_LOGI(TAG, "=== USB PD Status ===");
    
    if (status.contract_valid) {
        const char *types[] = {"Fixed", "Variable", "Battery", "PPS"};
        ESP_LOGI(TAG, "Contract: ACTIVE");
        ESP_LOGI(TAG, "Type: %s Supply", types[status.supply_type & 0x03]);
        ESP_LOGI(TAG, "Voltage: %d mV (%.1fV)", status.voltage_mv, status.voltage_mv / 1000.0f);
        ESP_LOGI(TAG, "Current: %d mA (%.1fA)", status.current_ma, status.current_ma / 1000.0f);
        ESP_LOGI(TAG, "Power: %.1fW", (status.voltage_mv / 1000.0f) * (status.current_ma / 1000.0f));
        
        if (status.object_position > 0) {
            ESP_LOGI(TAG, "PDO Position: %d", status.object_position);
        }
        if (status.capability_mismatch) {
            ESP_LOGW(TAG, "Capability Mismatch!");
        }
    } else {
        ESP_LOGI(TAG, "Contract: NONE");
        ESP_LOGI(TAG, "Defaulting to 5V USB");
    }
    
    ESP_LOGI(TAG, "=====================");
}

static bool selftest(){
    bool success = true;
    ESP_LOGI(TAG, "Starting self-test...");

    if(i2c_check_device(PD_ADDRESS)){
        ESP_LOGI(TAG, "PD Controller OK at 0x%02X", PD_ADDRESS);
    } else {
        ESP_LOGE(TAG, "PD Controller MISSING at 0x%02X", PD_ADDRESS);
        success = false;
    }   

    if(i2c_check_device(POWER_MONITOR_ADDRESS)){
        ESP_LOGI(TAG, "Power Monitor OK at 0x%02X", POWER_MONITOR_ADDRESS);
    } else {
        ESP_LOGE(TAG, "Power Monitor MISSING at 0x%02X", POWER_MONITOR_ADDRESS);
        success = false;
    }

    return success;
}

void app_main(void) {
    led_init();
    setupHeater();
    i2c_master_init();
    max31865_init();
    i2c_scan();
    ads1220_init();
    if(!selftest()){
        while(1){
            led_set_color(15, 255, 0, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_set_color(15, 0, 0, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
    
    // Wait for PD negotiation to complete
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Print initial status
    pd_print_status();
    
    while (1) {
        // LED color based on voltage
        uint16_t voltage = pd_get_voltage_mv();
        max31865_print_status();
        
        if (voltage >= 15000) {
            led_set_color(15, 0, 0, 255);   // Blue = 15V+
            // gpio_set_level(HEATER_PWM, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            ina260_print_status();
        } else {
            led_set_color(15, 255, 255, 0); // Yellow = unknown
            pd_print_status();
        }
        
        
        
        // ads1220_print_status();
    }
}
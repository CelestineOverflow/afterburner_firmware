#include "ina260.h"
#include "cJSON.h"
/* Registers */
#define INA260_REG_CONFIG   0x00
#define INA260_REG_CURRENT  0x01  // 1.25 mA/LSB
#define INA260_REG_VOLTAGE  0x02  // 1.25 mV/LSB
#define INA260_REG_POWER    0x03  // 10 mW/LSB

#define I2C_TIMEOUT_MS 100

static esp_err_t ina260_read_reg(
    ina260_t *dev,
    uint8_t reg,
    uint16_t *value
)
{
    uint8_t data[2];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd,
        (dev->i2c_addr << 1) | I2C_MASTER_WRITE,
        true
    );
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd,
        (dev->i2c_addr << 1) | I2C_MASTER_READ,
        true
    );
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        dev->i2c_port,
        cmd,
        pdMS_TO_TICKS(I2C_TIMEOUT_MS)
    );

    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *value = (data[0] << 8) | data[1];
    }

    return ret;
}

esp_err_t ina260_init(ina260_t *dev)
{
    if (dev->alert_pin != GPIO_NUM_NC) {
        gpio_config_t io_conf = {
            .pin_bit_mask = 1ULL << dev->alert_pin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
    }

    return ESP_OK;
}

esp_err_t ina260_get_voltage_mv(ina260_t *dev, int32_t *mv)
{
    uint16_t raw;
    esp_err_t ret = ina260_read_reg(dev, INA260_REG_VOLTAGE, &raw);
    if (ret == ESP_OK) {
        *mv = (int32_t)raw * 125 / 100; // 1.25 mV/LSB
    }
    return ret;
}

esp_err_t ina260_get_current_ma(ina260_t *dev, int32_t *ma)
{
    uint16_t raw;
    esp_err_t ret = ina260_read_reg(dev, INA260_REG_CURRENT, &raw);
    if (ret == ESP_OK) {
        *ma = (int16_t)raw * 125 / 100; // signed, 1.25 mA/LSB
    }
    return ret;
}

esp_err_t ina260_get_power_mw(ina260_t *dev, int32_t *mw)
{
    uint16_t raw;
    esp_err_t ret = ina260_read_reg(dev, INA260_REG_POWER, &raw);
    if (ret == ESP_OK) {
        *mw = (int32_t)raw * 10; // 10 mW/LSB
    }
    return ret;
}


cJSON * report_ina260_json(int32_t voltage_mv, int32_t current_ma, int32_t power_mw)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "voltage_mv", voltage_mv);
    cJSON_AddNumberToObject(json, "current_ma", current_ma);
    cJSON_AddNumberToObject(json, "power_mw", power_mw);
    return json;
}

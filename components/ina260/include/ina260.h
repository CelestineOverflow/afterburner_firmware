#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;        // usually 0x40
    gpio_num_t alert_pin;    // GPIO_NUM_NC if unused
} ina260_t;

/* Init INA260 (optional alert pin config) */
esp_err_t ina260_init(ina260_t *dev);

/* Measurements */
esp_err_t ina260_get_voltage_mv(ina260_t *dev, int32_t *mv);
esp_err_t ina260_get_current_ma(ina260_t *dev, int32_t *ma);
esp_err_t ina260_get_power_mw(ina260_t *dev, int32_t *mw);

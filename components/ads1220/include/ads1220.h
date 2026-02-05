#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "cJSON.h"

typedef struct {
    spi_host_device_t spi_host;
    spi_device_handle_t spi;
    gpio_num_t cs_pin;
    gpio_num_t drdy_pin;
    int clock_hz;
    uint8_t gain; // store configured gain for scaling
} ads1220_t;

/* Init ADS1220 (SPI bus must already be initialized) */
esp_err_t ads1220_init(ads1220_t *dev);

/* Read raw 24-bit signed value */
esp_err_t ads1220_read_raw(ads1220_t *dev, int32_t *raw);

/* Convert raw value to voltage */
float ads1220_raw_to_voltage(ads1220_t *dev, int32_t raw);
cJSON * report_loadcell_json(int32_t raw, int32_t zero, float multiplier);
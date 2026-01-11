#pragma once

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

typedef struct {
    spi_host_device_t spi_host;
    spi_device_handle_t spi;
    gpio_num_t data_ready;
    int cs_pin;

    float rref;      // 430.0 (PT100) or 4300.0 (PT1000)
    float rtd_nominal; // 100.0 or 1000.0

    bool three_wire;
    int clock_hz;
} max31865_t;

/* Init MAX31865 (SPI bus must already be initialized) */
esp_err_t max31865_init(max31865_t *dev);

/* Read temperature in °C */
esp_err_t max31865_get_temperature(max31865_t *dev, float *temp_c);

/* Read raw RTD value */
esp_err_t max31865_read_rtd_raw(max31865_t *dev, uint16_t *rtd);

/* Fault handling */
uint8_t max31865_read_fault(max31865_t *dev);
void max31865_clear_fault(max31865_t *dev);

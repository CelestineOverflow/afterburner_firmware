#include "max31865.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

/* Registers */
#define MAX31865_REG_CONFIG     0x00
#define MAX31865_REG_RTD_MSB    0x01
#define MAX31865_REG_FAULT      0x07

/* Config bits */
#define MAX31865_CFG_VBIAS      0x80
#define MAX31865_CFG_AUTO       0x40
#define MAX31865_CFG_1SHOT      0x20
#define MAX31865_CFG_3WIRE      0x10
#define MAX31865_CFG_FAULT_CLR  0x02
#define MAX31865_CFG_50HZ       0x01

/* Callendar–Van Dusen (PT100/1000) */
#define RTD_A 3.9083e-3f
#define RTD_B -5.775e-7f




static esp_err_t max31865_write_reg(max31865_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg | 0x80, value };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t max31865_read_reg(max31865_t *dev, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = { reg & 0x7F, 0x00 };
    uint8_t rx[2] = {0};

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    if (ret == ESP_OK) {
        *value = rx[1];
    }
    return ret;
}

esp_err_t max31865_read_rtd_raw(max31865_t *dev, uint16_t *rtd)
{
    uint8_t tx[3] = { MAX31865_REG_RTD_MSB & 0x7F, 0x00, 0x00 };
    uint8_t rx[3] = {0};

    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    if (ret == ESP_OK) {
        *rtd = ((rx[1] << 8) | rx[2]) >> 1;
    }
    return ret;
}

uint8_t max31865_read_fault(max31865_t *dev)
{
    uint8_t fault = 0;
    max31865_read_reg(dev, MAX31865_REG_FAULT, &fault);
    return fault;
}

void max31865_clear_fault(max31865_t *dev)
{
    uint8_t cfg;
    max31865_read_reg(dev, MAX31865_REG_CONFIG, &cfg);
    cfg |= MAX31865_CFG_FAULT_CLR;
    max31865_write_reg(dev, MAX31865_REG_CONFIG, cfg);
}

static float rtd_raw_to_resistance(max31865_t *dev, uint16_t raw)
{
    return ((float)raw * dev->rref) / 32768.0f;
}

static float resistance_to_temp(float r, float r0)
{
    float Z1 = -RTD_A;
    float Z2 = RTD_A * RTD_A - (4.0f * RTD_B);
    float Z3 = (4.0f * RTD_B) / r0;
    float Z4 = 2.0f * RTD_B;

    float temp = Z2 + (Z3 * r);
    return (sqrtf(temp) + Z1) / Z4;
}

esp_err_t max31865_get_temperature(max31865_t *dev, float *temp_c)
{
    uint16_t raw;
    esp_err_t ret = max31865_read_rtd_raw(dev, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    float resistance = rtd_raw_to_resistance(dev, raw);
    *temp_c = resistance_to_temp(resistance, dev->rtd_nominal);
    return ESP_OK;
}

esp_err_t max31865_init(max31865_t *dev)
{
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = dev->clock_hz,
        .mode = 1,
        .spics_io_num = dev->cs_pin,
        .queue_size = 1,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
    };

    esp_err_t ret = spi_bus_add_device(dev->spi_host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t cfg = MAX31865_CFG_VBIAS |
                  MAX31865_CFG_AUTO  |
                  MAX31865_CFG_50HZ;

    if (dev->three_wire) {
        cfg |= MAX31865_CFG_3WIRE;
    }

    ret = max31865_write_reg(dev, MAX31865_REG_CONFIG, cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

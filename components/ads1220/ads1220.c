#include "ads1220.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Commands */
#define ADS1220_CMD_RESET     0x06
#define ADS1220_CMD_START     0x08
#define ADS1220_CMD_RDATA     0x10
#define ADS1220_CMD_RREG      0x20
#define ADS1220_CMD_WREG      0x40

/* Registers */
#define ADS1220_REG0 0x00
#define ADS1220_REG1 0x01
#define ADS1220_REG2 0x02
#define ADS1220_REG3 0x03

/* Config bits (kept simple / identical to your setup) */
#define ADS1220_MUX_AIN0_AIN1 (0x03 << 4)
#define ADS1220_GAIN_128     (0x07 << 1)
#define ADS1220_DR_20SPS     (0x00 << 5)
#define ADS1220_MODE_NORMAL  (0x00 << 3)
#define ADS1220_CONV_CONT    (0x01 << 2)
#define ADS1220_VREF_INT     (0x03 << 6)
#define ADS1220_FIR_50_60    (0x01 << 4)

static esp_err_t ads1220_send_cmd(ads1220_t *dev, uint8_t cmd)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    return spi_device_transmit(dev->spi, &t);
}

static esp_err_t ads1220_write_reg(ads1220_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { ADS1220_CMD_WREG | (reg << 2), value };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    return spi_device_transmit(dev->spi, &t);
}

static bool ads1220_wait_drdy(ads1220_t *dev, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    while (gpio_get_level(dev->drdy_pin)) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            return false;
        }
        vTaskDelay(1);
    }
    return true;
}

esp_err_t ads1220_read_raw(ads1220_t *dev, int32_t *raw)
{
    if (!ads1220_wait_drdy(dev, 100)) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t tx[4] = { ADS1220_CMD_RDATA, 0, 0, 0 };
    uint8_t rx[4] = {0};

    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    if (ret != ESP_OK) {
        return ret;
    }

    int32_t value = ((int32_t)rx[1] << 16) |
                    ((int32_t)rx[2] << 8)  |
                     (int32_t)rx[3];

    if (value & 0x800000) {
        value |= 0xFF000000;
    }

    *raw = value;
    return ESP_OK;
}

float ads1220_raw_to_voltage(ads1220_t *dev, int32_t raw)
{
    const float vref = 2.048f;
    return ((float)raw * vref) / (8388608.0f * dev->gain);
}

esp_err_t ads1220_init(ads1220_t *dev)
{
    /* DRDY pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << dev->drdy_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    /* SPI device */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = dev->clock_hz,
        .mode = 1,
        .spics_io_num = dev->cs_pin,
        .queue_size = 1,
    };

    esp_err_t ret = spi_bus_add_device(dev->spi_host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        return ret;
    }

    ads1220_send_cmd(dev, ADS1220_CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    ads1220_write_reg(dev, ADS1220_REG0, 0x36);

    ads1220_write_reg(dev, ADS1220_REG1, 0x4);

    ads1220_write_reg(dev, ADS1220_REG2, 0x98);

    ads1220_write_reg(dev, ADS1220_REG3, 0x00);

    ads1220_send_cmd(dev, ADS1220_CMD_START);

    dev->gain = 128;
    return ESP_OK;
}

#include "ads1220.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "cJSON.h"

/* Commands */
#define ADS1220_CMD_RESET     0x06
#define ADS1220_CMD_START     0x08
#define ADS1220_CMD_RDATA     0x10
#define ADS1220_CMD_RREG      0x20
#define ADS1220_CMD_WREG      0x40

/* Register addresses */
#define ADS1220_REG0          0x00
#define ADS1220_REG1          0x01
#define ADS1220_REG2          0x02
#define ADS1220_REG3          0x03

/*
 * Register 0: Input MUX, Gain, PGA
 */
#define ADS1220_MUX_AIN0_AIN1   (0x00 << 4)
#define ADS1220_MUX_AIN0_AIN2   (0x01 << 4)
#define ADS1220_MUX_AIN0_AIN3   (0x02 << 4)
#define ADS1220_MUX_AIN1_AIN2   (0x03 << 4)
#define ADS1220_MUX_AIN1_AIN3   (0x04 << 4)
#define ADS1220_MUX_AIN2_AIN3   (0x05 << 4)
#define ADS1220_MUX_AIN1_AIN0   (0x06 << 4)
#define ADS1220_MUX_AIN3_AIN2   (0x07 << 4)
#define ADS1220_MUX_AIN0_AVSS   (0x08 << 4)
#define ADS1220_MUX_AIN1_AVSS   (0x09 << 4)
#define ADS1220_MUX_AIN2_AVSS   (0x0A << 4)
#define ADS1220_MUX_AIN3_AVSS   (0x0B << 4)

#define ADS1220_GAIN_1          (0x00 << 1)
#define ADS1220_GAIN_2          (0x01 << 1)
#define ADS1220_GAIN_4          (0x02 << 1)
#define ADS1220_GAIN_8          (0x03 << 1)
#define ADS1220_GAIN_16         (0x04 << 1)
#define ADS1220_GAIN_32         (0x05 << 1)
#define ADS1220_GAIN_64         (0x06 << 1)
#define ADS1220_GAIN_128        (0x07 << 1)

#define ADS1220_PGA_BYPASS      (0x01 << 0)

/*
 * Register 1: Data rate, Mode, Conversion, Temp sensor
 */
#define ADS1220_DR_20SPS        (0x00 << 5)
#define ADS1220_DR_45SPS        (0x01 << 5)
#define ADS1220_DR_90SPS        (0x02 << 5)
#define ADS1220_DR_175SPS       (0x03 << 5)
#define ADS1220_DR_330SPS       (0x04 << 5)
#define ADS1220_DR_600SPS       (0x05 << 5)
#define ADS1220_DR_1000SPS      (0x06 << 5)

#define ADS1220_MODE_NORMAL     (0x00 << 4)
#define ADS1220_MODE_TURBO      (0x01 << 4)

#define ADS1220_CM_SINGLE       (0x00 << 3)
#define ADS1220_CM_CONTINUOUS   (0x01 << 3)

#define ADS1220_TS_DISABLED     (0x00 << 2)
#define ADS1220_TS_ENABLED      (0x01 << 2)

#define ADS1220_BCS_OFF         (0x00 << 1)
#define ADS1220_BCS_ON          (0x01 << 1)

/*
 * Register 2: Voltage reference, FIR filter, Power switch, IDAC
 */
#define ADS1220_VREF_INT        (0x00 << 6)  /* 2.048V internal */
#define ADS1220_VREF_EXT_REFP0  (0x01 << 6)  /* External on REFP0/REFN0 */
#define ADS1220_VREF_AIN0_AIN3  (0x02 << 6)  /* AIN0/AIN3 as reference */
#define ADS1220_VREF_AVDD       (0x03 << 6)  /* Analog supply */

#define ADS1220_FIR_NONE        (0x00 << 4)
#define ADS1220_FIR_50_60       (0x01 << 4)
#define ADS1220_FIR_50HZ        (0x02 << 4)
#define ADS1220_FIR_60HZ        (0x03 << 4)

#define ADS1220_PSW_OPEN        (0x01 << 3)
#define ADS1220_PSW_CLOSED      (0x00 << 3)

#define ADS1220_IDAC_OFF        (0x00)
#define ADS1220_IDAC_10UA       (0x01)
#define ADS1220_IDAC_50UA       (0x02)
#define ADS1220_IDAC_100UA      (0x03)
#define ADS1220_IDAC_250UA      (0x04)
#define ADS1220_IDAC_500UA      (0x05)
#define ADS1220_IDAC_1000UA     (0x06)
#define ADS1220_IDAC_1500UA     (0x07)

/*
 * Register 3: IDAC routing, DRDY mode
 */
#define ADS1220_I1MUX_DISABLED  (0x00 << 5)
#define ADS1220_I1MUX_AIN0      (0x01 << 5)
#define ADS1220_I1MUX_AIN1      (0x02 << 5)
#define ADS1220_I1MUX_AIN2      (0x03 << 5)
#define ADS1220_I1MUX_AIN3      (0x04 << 5)
#define ADS1220_I1MUX_REFP0     (0x05 << 5)
#define ADS1220_I1MUX_REFN0     (0x06 << 5)

#define ADS1220_I2MUX_DISABLED  (0x00 << 2)
#define ADS1220_I2MUX_AIN0      (0x01 << 2)
#define ADS1220_I2MUX_AIN1      (0x02 << 2)
#define ADS1220_I2MUX_AIN2      (0x03 << 2)
#define ADS1220_I2MUX_AIN3      (0x04 << 2)
#define ADS1220_I2MUX_REFP0     (0x05 << 2)
#define ADS1220_I2MUX_REFN0     (0x06 << 2)

#define ADS1220_DRDY_DEDICATED  (0x00 << 1)
#define ADS1220_DRDY_DOUT       (0x01 << 1)

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

    /*
     * REG0 = 0x36: AIN1 vs AIN2, Gain=8, PGA enabled
     */
    ads1220_write_reg(dev, ADS1220_REG0,
        ADS1220_MUX_AIN1_AIN2 | ADS1220_GAIN_8);

    /*
     * REG1 = 0x04: 20 SPS, normal mode, single-shot, temp sensor enabled
     * Note: TS=1 seems odd for load cell but this config works
     */
    ads1220_write_reg(dev, ADS1220_REG1,
        ADS1220_DR_20SPS | ADS1220_MODE_NORMAL | ADS1220_CM_SINGLE | ADS1220_TS_ENABLED);

    /*
     * REG2 = 0x98: External ref on AIN0/AIN3, 50/60Hz rejection, low-side switch open
     */
    ads1220_write_reg(dev, ADS1220_REG2,
        ADS1220_VREF_AIN0_AIN3 | ADS1220_FIR_50_60 | ADS1220_PSW_OPEN | ADS1220_IDAC_OFF);

    /*
     * REG3 = 0x00: IDACs disabled, DRDY on dedicated pin
     */
    ads1220_write_reg(dev, ADS1220_REG3,
        ADS1220_I1MUX_DISABLED | ADS1220_I2MUX_DISABLED | ADS1220_DRDY_DEDICATED);

    ads1220_send_cmd(dev, ADS1220_CMD_START);

    dev->gain = 4;
    return ESP_OK;
}

cJSON *report_loadcell_json(int32_t raw, int32_t zero, float multiplier)
{
    cJSON *json = cJSON_CreateObject();
    float reading = (raw - zero) * multiplier;
    cJSON_AddNumberToObject(json, "loadcell", reading);
    return json;
}

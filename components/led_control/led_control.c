#include "led_control.h"

static void led_send_byte(led_ctrl_t *led, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(led->data_pin, (byte >> i) & 1);
        gpio_set_level(led->clk_pin, 1);
        gpio_set_level(led->clk_pin, 0);
    }
}

void led_init(led_ctrl_t *led)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << led->clk_pin) | (1ULL << led->data_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);

    gpio_set_level(led->clk_pin, 0);
    gpio_set_level(led->data_pin, 0);
}

void led_set_color(
    led_ctrl_t *led,
    uint8_t brightness,
    uint8_t r,
    uint8_t g,
    uint8_t b
)
{
    // Start frame
    for (int i = 0; i < 4; i++) {
        led_send_byte(led, 0x00);
    }

    // LED frame
    led_send_byte(led, 0xE0 | (brightness & 0x1F));
    led_send_byte(led, b);
    led_send_byte(led, g);
    led_send_byte(led, r);

    // End frame
    for (int i = 0; i < 4; i++) {
        led_send_byte(led, 0xFF);
    }
}

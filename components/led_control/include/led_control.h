#pragma once

#include <stdint.h>
#include "driver/gpio.h"

typedef struct {
    gpio_num_t clk_pin;
    gpio_num_t data_pin;
} led_ctrl_t;

/**
 * Initialize LED control with given pins
 */
void led_init(led_ctrl_t *led);

/**
 * Set LED color and brightness
 * brightness: 0–31
 */
void led_set_color(
    led_ctrl_t *led,
    uint8_t brightness,
    uint8_t r,
    uint8_t g,
    uint8_t b
);

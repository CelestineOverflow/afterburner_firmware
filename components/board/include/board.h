#pragma once

// Led
#define LED_CLOCK 4
#define LED_DATA_IN 5
#define NUM_LEDS 1

// Power Delivery TPS25730
#define PD_POWER_ON 6
#define PD_ADDRESS 0x20

// TPS25730 Register Addresses
#define TPS25730_REG_ACTIVE_CONTRACT_PDO 0x34 // Length 6
#define TPS25730_REG_ACTIVE_CONTRACT_RDO 0x35 // Length 4

// Heater Mosfet PWM
#define HEATER_PWM 7

// I2C Pins
#define SDA 18
#define SCL 19
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TIMEOUT_MS 10


// SPI
#define MOSI 22
#define MISO 23
#define SCK 21

// MAX31865
#define TEMP_CS 20
#define TEMP_DATA_READY 3

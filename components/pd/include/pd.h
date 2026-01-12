#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "cJSON.h"

typedef struct {
    bool contract_valid;
    bool capability_mismatch;
    uint8_t object_position;
    uint16_t voltage_mv;
    uint16_t current_ma;
    uint8_t supply_type;
} pd_status_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    pd_status_t last_valid_status;
    uint8_t consecutive_errors;
    bool has_valid_cache;
} pd_t;

esp_err_t pd_init(pd_t *pd);
esp_err_t pd_read_status(pd_t *pd, pd_status_t *status);
uint16_t pd_get_voltage_mv(pd_t *pd);
bool pd_is_contract_active(pd_t *pd);
void pd_print_status(pd_t *pd);
cJSON * report_pd_json(uint16_t voltage_mv);
#include "pd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

// TPS25730 Registers
#define TPS25730_REG_ACTIVE_CONTRACT_PDO 0x34
#define TPS25730_REG_ACTIVE_CONTRACT_RDO 0x35
#define MAX_CONSECUTIVE_ERRORS 5

static const char *TAG = "PD";

/* --------------------------------------------------------
 * Low-level register read with retry
 * -------------------------------------------------------- */
static esp_err_t pd_read_register(pd_t *pd, uint8_t reg, uint8_t *data, uint8_t expected_len)
{
    uint8_t buffer[16];
    esp_err_t ret = ESP_FAIL;
    uint8_t total = expected_len + 1;
    
    for (int attempt = 0; attempt < 3; attempt++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (pd->i2c_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (pd->i2c_addr << 1) | I2C_MASTER_READ, true);
        for (int i = 0; i < total - 1; i++)
            i2c_master_read_byte(cmd, &buffer[i], I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &buffer[total - 1], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        ret = i2c_master_cmd_begin(pd->i2c_port, cmd, pdMS_TO_TICKS(200));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK && (buffer[0] == expected_len || buffer[0] == 0)) {
            memcpy(data, &buffer[1], expected_len);
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return ret;
}

/* --------------------------------------------------------
 * Public API
 * -------------------------------------------------------- */
esp_err_t pd_init(pd_t *pd)
{
    if (!pd)
        return ESP_ERR_INVALID_ARG;
    
    // Initialize error tracking and cache
    pd->consecutive_errors = 0;
    pd->has_valid_cache = false;
    memset(&pd->last_valid_status, 0, sizeof(pd_status_t));
    pd->last_valid_status.voltage_mv = 5000;  // Default to 5V
    
    ESP_LOGI(TAG, "PD initialized (addr=0x%02X)", pd->i2c_addr);
    return ESP_OK;
}

esp_err_t pd_read_status(pd_t *pd, pd_status_t *status)
{
    if (!pd || !status)
        return ESP_ERR_INVALID_ARG;

    uint8_t pdo_data[6] = {0};
    uint8_t rdo_data[4] = {0};
    pd_status_t temp_status;
    memset(&temp_status, 0, sizeof(pd_status_t));

    // --- Try to read ACTIVE_CONTRACT_PDO ---
    if (pd_read_register(pd, TPS25730_REG_ACTIVE_CONTRACT_PDO, pdo_data, 6) != ESP_OK) {
        pd->consecutive_errors++;
        
        if (pd->consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "Failed to read PDO %d times - giving up", MAX_CONSECUTIVE_ERRORS);
            temp_status.contract_valid = false;
            temp_status.voltage_mv = 5000;
            *status = temp_status;
            return ESP_FAIL;
        }
        
        // Use cached value
        if (pd->has_valid_cache) {
            ESP_LOGD(TAG, "Read failed (error %d/%d) - using cached value", 
                     pd->consecutive_errors, MAX_CONSECUTIVE_ERRORS);
            *status = pd->last_valid_status;
            return ESP_OK;
        } else {
            ESP_LOGW(TAG, "Read failed and no cache available");
            temp_status.contract_valid = false;
            temp_status.voltage_mv = 5000;
            *status = temp_status;
            return ESP_FAIL;
        }
    }

    // Read succeeded - reset error counter
    pd->consecutive_errors = 0;

    uint32_t pdo =
        pdo_data[0] |
        (pdo_data[1] << 8) |
        (pdo_data[2] << 16) |
        (pdo_data[3] << 24);

    if (pdo == 0) {
        temp_status.contract_valid = false;
        temp_status.voltage_mv = 5000;
        *status = temp_status;
        pd->last_valid_status = temp_status;
        pd->has_valid_cache = true;
        return ESP_OK;
    }

    temp_status.supply_type = (pdo >> 30) & 0x03;
    switch (temp_status.supply_type) {
        case 0: // Fixed
            temp_status.voltage_mv = ((pdo >> 10) & 0x3FF) * 50;
            temp_status.current_ma = (pdo & 0x3FF) * 10;
            break;
        case 1: // Variable
        case 2: // Battery
            temp_status.voltage_mv = ((pdo >> 20) & 0x3FF) * 50;
            temp_status.current_ma = (pdo & 0x3FF) * 10;
            break;
        case 3: // PPS
            temp_status.voltage_mv = ((pdo >> 17) & 0xFF) * 100;
            temp_status.current_ma = (pdo & 0x7F) * 50;
            break;
        default:
            temp_status.voltage_mv = 5000;
            break;
    }
    temp_status.contract_valid = true;

    // --- Read ACTIVE_CONTRACT_RDO (best effort) ---
    if (pd_read_register(pd, TPS25730_REG_ACTIVE_CONTRACT_RDO, rdo_data, 4) == ESP_OK) {
        uint32_t rdo =
            rdo_data[0] |
            (rdo_data[1] << 8) |
            (rdo_data[2] << 16) |
            (rdo_data[3] << 24);
        
        if (rdo != 0) {
            temp_status.object_position = (rdo >> 28) & 0x07;
            temp_status.capability_mismatch = (rdo >> 26) & 0x01;
            uint16_t rdo_current = ((rdo >> 10) & 0x3FF) * 10;
            if (rdo_current)
                temp_status.current_ma = rdo_current;
        }
    }

    // Update cache with new valid data
    pd->last_valid_status = temp_status;
    pd->has_valid_cache = true;
    *status = temp_status;
    
    return ESP_OK;
}

uint16_t pd_get_voltage_mv(pd_t *pd)
{
    pd_status_t status;
    pd_read_status(pd, &status);
    return status.voltage_mv;
}

bool pd_is_contract_active(pd_t *pd)
{
    pd_status_t status;
    if (pd_read_status(pd, &status) != ESP_OK)
        return false;
    return status.contract_valid;
}

void pd_print_status(pd_t *pd)
{
    pd_status_t status;
    esp_err_t ret = pd_read_status(pd, &status);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PD status unavailable");
        return;
    }

    ESP_LOGI(TAG, "=== USB PD Status ===");
    if (pd->consecutive_errors > 0) {
        ESP_LOGW(TAG, "Using cached data (errors: %d/%d)", 
                 pd->consecutive_errors, MAX_CONSECUTIVE_ERRORS);
    }
    
    if (status.contract_valid) {
        const char *types[] = {"Fixed", "Variable", "Battery", "PPS"};
        ESP_LOGI(TAG, "Contract: ACTIVE");
        ESP_LOGI(TAG, "Type: %s", types[status.supply_type & 0x03]);
        ESP_LOGI(TAG, "Voltage: %d mV (%.1fV)",
                 status.voltage_mv,
                 status.voltage_mv / 1000.0f);
        ESP_LOGI(TAG, "Current: %d mA (%.1fA)",
                 status.current_ma,
                 status.current_ma / 1000.0f);
        ESP_LOGI(TAG, "Power: %.1f W",
                 (status.voltage_mv / 1000.0f) * (status.current_ma / 1000.0f));
        if (status.object_position)
            ESP_LOGI(TAG, "PDO Position: %d", status.object_position);
        if (status.capability_mismatch)
            ESP_LOGW(TAG, "Capability Mismatch");
    } else {
        ESP_LOGI(TAG, "Contract: NONE (default 5V)");
    }
    ESP_LOGI(TAG, "=====================");
}
/*
 * sht40.c
 *
 *  Created on: Sep 7, 2026
 *      Author: neeve
 */

#include "sht40.h"

// CRC-8 validation (Polynomial: 0x31, Init: 0xFF)
static uint8_t SHT40_CheckCRC(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

HAL_StatusTypeDef SHT40_Read(I2C_HandleTypeDef *hi2c2, SHT40_Data_t *data) {
    uint8_t cmd = SHT40_CMD_MEASURE_HIGH;
    uint8_t rx_buffer[6];
    HAL_StatusTypeDef status;

    // 1. Send measurement trigger command
    status = HAL_I2C_Master_Transmit(hi2c2, SHT40_I2C_ADDR, &cmd, 1, 100);
    if (status != HAL_OK) {
        return status;
    }

    // 2. Wait for measurement conversion (High precision max is ~8.3 ms)
    HAL_Delay(10);

    // 3. Read 6 data bytes back
    status = HAL_I2C_Master_Receive(hi2c2, SHT40_I2C_ADDR, rx_buffer, 6, 100);
    if (status != HAL_OK) {
        return status;
    }

    // 4. Validate CRCs
    if (SHT40_CheckCRC(&rx_buffer[0], 2) != rx_buffer[2]) {
        return HAL_ERROR; // Temperature CRC mismatch
    }
    if (SHT40_CheckCRC(&rx_buffer[3], 2) != rx_buffer[5]) {
        return HAL_ERROR; // Humidity CRC mismatch
    }

    // 5. Convert raw ticks into physical values
    uint16_t raw_temp = ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];
    uint16_t raw_humi = ((uint16_t)rx_buffer[3] << 8) | rx_buffer[4];

    // Formulas per Sensirion SHT4x Datasheet
    data->temperature = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    data->humidity    = -6.0f + 125.0f * ((float)raw_humi / 65535.0f);

    // Clamp humidity between 0% and 100%
    if (data->humidity < 0.0f)   data->humidity = 0.0f;
    if (data->humidity > 100.0f) data->humidity = 100.0f;

    return HAL_OK;
}


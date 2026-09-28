/*
 * lis3dh_driver.c
 *
 *  Created on: Mar 10, 2025
 *      Author: 91748
 */

// Includes
#include "lis3dh_driver.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"

// Macros
#define OUT_X_L 0x28
#define OUT_Y_L 0x2A
#define OUT_Z_L 0x2C
#define WHO_AM_I (uint8_t *)0x0F

#define FILTER_DEPTH 5

uint16_t window_history[FILTER_DEPTH] = {1, 1, 1, 1, 1}; // Default to Idle (ID 1)
uint8_t hist_index = 0;

extern I2C_HandleTypeDef hi2c2;
uint8_t who_am_i = 0x0F;
uint8_t CTRL_REG_1 = 0x20 ;
uint8_t var = 0;

/*	OUT_X_L (28h), OUT_X_H (29h)
 * 	OUT_Y_L (2Ah), OUT_Y_H (2Bh)
 * 	OUT_Z_L (2Ch), OUT_Z_H (2Dh)
 * */

HAL_StatusTypeDef LIS3DH_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i = 0;
    uint8_t reg_val;

    /* 1. Check WHO_AM_I */
    if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, LIS3DH_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (who_am_i != LIS3DH_WHO_AM_I_VAL) // Must match 0x33
    {
        return HAL_ERROR;
    }

    /* 2. Configure CTRL_REG1: ODR 400Hz, all axes enabled (0x77) */
    reg_val = 0x77;
    if (HAL_I2C_Mem_Write(hi2c, LIS3DH_ADDR, LIS3DH_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 3. Configure CTRL_REG4: BDU enabled (0x80) | +/- 2g scale (0x00) | High-Resolution 12-bit (0x08) -> 0x88 */
    reg_val = 0x88;
    if (HAL_I2C_Mem_Write(hi2c, LIS3DH_ADDR, LIS3DH_CTRL_REG4, I2C_MEMADD_SIZE_8BIT, &reg_val, 1, 100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef LIS3DH_ReadWindow(I2C_HandleTypeDef *hi2c, float *buffer, uint16_t sample_count)
{
    uint8_t status = 0;
    uint8_t raw_data[6];

    for (uint16_t i = 0; i < sample_count; i++)
    {
        /* 1. Wait for next DRDY sample (ZYXDA = 0x08) */
        do {
            if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, 0x27, I2C_MEMADD_SIZE_8BIT, &status, 1, 50) != HAL_OK)
            {
                return HAL_ERROR;
            }
        } while ((status & 0x08) == 0);

        /* 2. 0x28 | 0x80 forces auto-increment through OUT_X_L to OUT_Z_H */
        if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, (0x28 | 0x80), I2C_MEMADD_SIZE_8BIT, raw_data, 6, 50) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* 3. 12-bit signed shift */
        int16_t x_raw = (int16_t)(((uint16_t)raw_data[1] << 8) | raw_data[0]) >> 4;
        int16_t y_raw = (int16_t)(((uint16_t)raw_data[3] << 8) | raw_data[2]) >> 4;
        int16_t z_raw = (int16_t)(((uint16_t)raw_data[5] << 8) | raw_data[4]) >> 4;

        /* 4. Match the 0.001 multiplier from Python training */
        buffer[i * 3 + 0] = (float)x_raw * 0.001f;
        buffer[i * 3 + 1] = (float)y_raw * 0.001f;
        buffer[i * 3 + 2] = (float)z_raw * 0.001f;
    }

    return HAL_OK;
}

HAL_StatusTypeDef LIS3DH_StreamXYZ(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    uint8_t status = 0;
    uint8_t raw_i2c[6];

    /* 1. Check Data Ready (ZYXDA bit in STATUS_REG 0x27) */
    if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, LIS3DH_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status, 1, 10) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if ((status & LIS3DH_STATUS_ZYXDA) == 0)
    {
        return HAL_BUSY; // No new sample available yet
    }

    /* 2. Read 6 raw acceleration bytes starting from OUT_X_L with auto-increment (0x80) */
    if (HAL_I2C_Mem_Read(hi2c, LIS3DH_ADDR, (LIS3DH_OUT_X_L | LIS3DH_AUTO_INCREMENT),
                         I2C_MEMADD_SIZE_8BIT, raw_i2c, 6, 10) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 3. Convert 12-bit High-Resolution values (left-justified -> shift right 4) */
    int16_t x_axis = (int16_t)((raw_i2c[1] << 8) | raw_i2c[0]) >> 4;
    int16_t y_axis = (int16_t)((raw_i2c[3] << 8) | raw_i2c[2]) >> 4;
    int16_t z_axis = (int16_t)((raw_i2c[5] << 8) | raw_i2c[4]) >> 4;

    /* 4. Capture current timestamp in milliseconds */
    uint32_t timestamp_ms = HAL_GetTick();

    /* 5. Structured 12-byte binary frame:
     * [0xAA, 0x55, Time_B0, Time_B1, Time_B2, Time_B3, XL, XH, YL, YH, ZL, ZH]
     */
    uint8_t packet[12];
    packet[0]  = 0xAA;  // Sync header 1
    packet[1]  = 0x55;  // Sync header 2

    /* Timestamp (32-bit Little-Endian) */
    packet[2]  = (uint8_t)(timestamp_ms & 0xFF);
    packet[3]  = (uint8_t)((timestamp_ms >> 8) & 0xFF);
    packet[4]  = (uint8_t)((timestamp_ms >> 16) & 0xFF);
    packet[5]  = (uint8_t)((timestamp_ms >> 24) & 0xFF);

    /* Aligned 16-bit values (Little-Endian) */
    packet[6]  = (uint8_t)(x_axis & 0xFF);
    packet[7]  = (uint8_t)((x_axis >> 8) & 0xFF);
    packet[8]  = (uint8_t)(y_axis & 0xFF);
    packet[9]  = (uint8_t)((y_axis >> 8) & 0xFF);
    packet[10] = (uint8_t)(z_axis & 0xFF);
    packet[11] = (uint8_t)((z_axis >> 8) & 0xFF);

    /* 6. Transmit binary frame over UART */
    return HAL_UART_Transmit(huart, packet, 12, 10);
}

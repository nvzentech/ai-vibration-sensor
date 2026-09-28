/*
 * sht40.h
 *
 *  Created on: Sep 7, 2026
 *      Author: neeve
 */

#ifndef INC_SHT40_H_
#define INC_SHT40_H_
#ifndef SHT40_H
#define SHT40_H

#include "stm32wlxx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define SHT40_I2C_ADDR          (0x44 << 1) // STM32 HAL requires 8-bit shifted address
#define SHT40_CMD_MEASURE_HIGH  0xFD        // High repeatability measurement command

typedef struct {
    float temperature; // In Celsius
    float humidity;    // In %RH
} SHT40_Data_t;

HAL_StatusTypeDef SHT40_Read(I2C_HandleTypeDef *hi2c, SHT40_Data_t *data);

#endif // SHT40_H



#endif /* INC_SHT40_H_ */

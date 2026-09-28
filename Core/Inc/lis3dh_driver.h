/*
 * accelerometer_driver.h
 *
 *  Created on: Mar 10, 2025
 *      Author: 91748
 */

//#ifndef INC_LIS3DH_DRIVER_H_
//#define INC_LIS3DH_DRIVER_H_
//
//#include  "main.h"
//#include  "stdbool.h"
//
////// Creating a type -->  AccValues_t
//typedef struct
//{
//	float x_accel ;
//	float y_accel ;
//	float z_accel ;
//}AccValues_t;
////
////void lis3dh_init(void) ;
////void lis3dh_setODR(uint8_t odr) ;
////void lis3dh_readData(void) ;
////void lis3dh_calc_value(uint16_t raw_value, float *final_value, bool isAccel);
////void  who_am_i_read(void);
////AccValues_t readAccelerationDataXYZ(void);
//
//typedef enum{
//	VIB_IDLE=0,
//	VIB_LOW,
//	VIB_MED,
//	VIB_HIGH
//}VibrationLevel_t;
//
//#define DATA_POINTS        2 // 256 samples = ~300ms window (fast & accurate)
//
//typedef struct {
//    float rms_total;
//    float peak_to_peak;
//    float peak_max;
//} VibFeatures_t;
//
//
//#include "stm32wlxx_hal.h"
//#include <stdint.h>
//
//#define LIS3DSH_ADDR       (0x1E << 1)   // Change to (0x1E << 1) if needed
//
//#define LIS3DSH_WHO_AM_I  0x0F
//#define LIS3DSH_CTRL_REG4 0x20
//#define LIS3DSH_CTRL_REG5 0x24
//
//#define LIS3DSH_OUT_X_L   0x28
//#define LIS3DSH_OUT_X_H   0x29
//#define LIS3DSH_OUT_Y_L   0x2A
//#define LIS3DSH_OUT_Y_H   0x2B
//#define LIS3DSH_OUT_Z_L   0x2C
//#define LIS3DSH_OUT_Z_H   0x2D
//
//HAL_StatusTypeDef LIS3DSH_Init(I2C_HandleTypeDef *hi2c);
//
//
//HAL_StatusTypeDef LIS3DSH_ReadXYZ(I2C_HandleTypeDef *hi2c,
//                                  float *x,
//								  float *y,
//								  float *z);
//
//
//HAL_StatusTypeDef LIS3DH_ReadWindow(I2C_HandleTypeDef *hi2c, float *buffer, uint16_t sample_count);
//
//HAL_StatusTypeDef LIS3DSH_StreamXYZ(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
//
//#endif /* INC_LIS3DH_DRIVER_H_ */


#ifndef INC_LIS3DH_DRIVER_H_
#define INC_LIS3DH_DRIVER_H_

#include "stm32wlxx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C Address: 0x18 if SDO/SA0 is GND, 0x19 if SDO/SA0 is VDD */
#define LIS3DH_ADDR                 (0x19 << 1)

#define LIS3DH_WHO_AM_I             0x0F
#define LIS3DH_WHO_AM_I_VAL         0x33

/* Control Registers */
#define LIS3DH_CTRL_REG1            0x20
#define LIS3DH_CTRL_REG4            0x23

/* Output Registers */
#define LIS3DH_OUT_X_L              0x28
#define LIS3DH_OUT_X_H              0x29
#define LIS3DH_OUT_Y_L              0x2A
#define LIS3DH_OUT_Y_H              0x2B
#define LIS3DH_OUT_Z_L              0x2C
#define LIS3DH_OUT_Z_H              0x2D

/* Auto-increment bit for multi-byte I2C read */
#define LIS3DH_AUTO_INCREMENT       0x80

#define LIS3DH_STATUS_REG           0x27
#define LIS3DH_STATUS_ZYXDA         0x08  // X, Y, and Z-axis new data available

/* Types */
typedef struct {
    float x_accel;
    float y_accel;
    float z_accel;
} AccValues_t;

typedef enum {
    VIB_IDLE = 0,
    VIB_LOW,
    VIB_MED,
    VIB_HIGH
} VibrationLevel_t;

typedef struct {
    float rms_total;
    float peak_to_peak;
    float peak_max;
} VibFeatures_t;

/* Function Prototypes */
HAL_StatusTypeDef LIS3DH_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LIS3DH_ReadXYZ(I2C_HandleTypeDef *hi2c, float *x, float *y, float *z);
HAL_StatusTypeDef LIS3DH_ReadWindow(I2C_HandleTypeDef *hi2c, float *buffer, uint16_t sample_count);
HAL_StatusTypeDef LIS3DH_StreamXYZ(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void lis3dh_read_data(uint8_t reg, float *final_value, bool IsAccel);

#endif /* INC_LIS3DH_DRIVER_H_ */

/*
 * lora_comm.h
 *
 *  Created on: Mar 22, 2025
 *      Author: 91748
 */

#ifndef INC_LORA_COMM_H_
#define INC_LORA_COMM_H_

// Includes
#include "globals.h"
#include "stdio.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "lora_app.h"
#include "gps_parser.h"
#include "gpio.h"
#include "stdlib.h"
#include "sys_app.h"

// Prototypes
int sendLoRaTxData(const char* payload , bool confirmed);
void sendLoRaPayload();
void updateGPSData();


#endif /* INC_LORA_COMM_H_ */

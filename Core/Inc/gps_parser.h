/*
 * gps_parser.h
 *
 *  Created on: Mar 22, 2025
 *      Author: 91748
 */

#ifndef INC_GPS_PARSER_H_
#define INC_GPS_PARSER_H_

// Includes
#include "globals.h"
#include "stdio.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"

// Variables


// Prototypes
char* packet_checking_fun(char* uart_rx_buffer);
void parse_nmea_sentence(const char *nmea_sentence, char *latitude, char *longitude);
float convertToDecimalDegrees(float);


#endif /* INC_GPS_PARSER_H_ */

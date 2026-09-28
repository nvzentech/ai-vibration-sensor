/*
 * lora_comm.c
 *
 *  Created on: Mar 22, 2025
 *      Author: 91748
 */

// Includes
#include "globals.h"
#include "lora_comm.h"
#include "adc_if.h"

#include "i2c.h"
#include "sht40.h"

#define APP_DATA_BUFFER_MAX_SIZE 128
#define LORA_SUCCESS  0
#define LORA_FAIL     -1

extern char uart_rx_buffer[];
extern SHT40_Data_t sht40_data;

bool gpsDataReady;
int flag = 0;

char latitude[16] = {0};
char longitude[16] = {0};
float lati = 0.0, longi = 0.0;

uint8_t AppDataBuffer[APP_DATA_BUFFER_MAX_SIZE];
LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

extern uint16_t current_vibration_state;
extern uint16_t id_class;

int sendLoRaTxData(const char* payload , bool confirmed)
{
	size_t len = strlen(payload);
	if (len >= APP_DATA_BUFFER_MAX_SIZE)
	{
		len = APP_DATA_BUFFER_MAX_SIZE - 1;
	}
	strncpy((char *)AppData.Buffer , payload , len);
	AppData.Buffer[len] = '\0';
	AppData.BufferSize = len;
	AppData.Port = 2 ;

	LmHandlerErrorStatus_t ret = LmHandlerSend(&AppData, confirmed ? LORAMAC_HANDLER_CONFIRMED_MSG : LORAMAC_HANDLER_UNCONFIRMED_MSG, false);

	if (ret == LORAMAC_HANDLER_SUCCESS)  															// Check if ACK is received or failed
	{
		printf("LoRa TX Confirmed!\r\n");
		HAL_GPIO_WritePin(GPIOA , GPIO_PIN_10 , GPIO_PIN_SET);				// Turn On BUZZER only after Acknowledgment
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA , GPIO_PIN_10 , GPIO_PIN_RESET);
		return LORA_SUCCESS;
	}
	else
	{
		printf("Handler return : %d\r\n",ret);
		printf("LoRa Tx Failed\r\n");
		return LORA_FAIL;
	}
}


/*	sendLoRaPayload()
 *  	Sends latest GPS + Temperature + Humidity + Battery Data over LoRa.
 *  */
void sendLoRaPayload()
{
	/* Don't try to send if we haven't joined the network yet */
	if (LmHandlerJoinStatus() != LORAMAC_HANDLER_SET)
	{
		printf("LoRa not joined yet, skipping send...\r\n");
		return;
	}

	/* Read battery voltage in millivolts and convert to volts */
	uint16_t battery_mv = SYS_GetBatteryLevel();
	float battery_v = (float)battery_mv / 1000.0f;

	/* Convert 0-254 battery scale to 0-100% percentage */
	uint8_t battery_pct = (uint8_t)(((uint32_t)GetBatteryLevel() * 100) / 254);

	uint8_t status;

	if (MP2667_ReadReg(0x07, &status) == HAL_OK)
	{
	    printf("Status = 0x%02X\r\n", status);
	}

	uint8_t chg = (status >> 3) & 0x03;

	switch(chg)
	{
	case 0:
	    printf("Not Charging\n");
	    break;

	case 1:
	    printf("Pre-charge\n");
	    break;

	case 2:
	    printf("Charging\n");
	    break;

	case 3:
	    printf("Charge Done\n");
	    break;

	default:
	    printf("Unknown\n");
	    break;
	}

	char payload[512] = {0};

//	if(gpsDataReady)
//	{
//		snprintf(payload , sizeof(payload) , "%.4f,%.4f,temp:%.2f,hum:%.2f,cs:%d,vib:%d" ,
//				lati , longi , my_sensor.temperature_C , my_sensor.humidity_RH , chg , current_vibration_state);
//	}
//	else
//	{
//		snprintf(payload , sizeof(payload) , "0.0000,0.0000,temp:%.2f,hum:%.2f,cs:%d,vib:%d" ,
//				my_sensor.temperature_C , my_sensor.humidity_RH , chg , current_vibration_state);
//	}

	if(gpsDataReady)
		{
			snprintf(payload, sizeof(payload), "%.4f,%.4f,temp:%.2f,hum:%.2f,cs:%d,vib:%d",
					lati, longi, sht40_data.temperature, sht40_data.humidity, chg, current_vibration_state);
		}
		else
		{
			snprintf(payload, sizeof(payload), "0.0000,0.0000,temp:%.2f,hum:%.2f,cs:%d,vib:%d",
					sht40_data.temperature, sht40_data.humidity, chg, current_vibration_state);
		}

	printf("*****Lora Payload*****:%s\n",payload);

 	    // Reset Acknowledgment flag
		if(sendLoRaTxData(payload , false) == LORA_SUCCESS)
		{
			printf("Data Sent successfully !!!!!!\r\n");
		}
		else
		{
			printf("LoRa Transmission Failed at initial send ...\r\n");
		}
}

/* Function that continuously acquires GPS Data */
void updateGPSData()
{
	char* whole_single_packet_data = packet_checking_fun(uart_rx_buffer); 		// Extract GNRMC sentence --> PacketCheckingFunction
	printf("GPS DATA  :  %s\r\n",whole_single_packet_data);
	parse_nmea_sentence(whole_single_packet_data, latitude, longitude); 		// Parse latitude and longitude --> Filtration : Lat , Long
	//printf("Parsed LATITUDE : %s , LONGITUDE : %s\r\n", latitude , longitude);	// Print latitude and longitude (for debugging)

	// Ensure Valid GPS data is stored
	if(latitude[0] != '\0' && longitude[0] != '\0' && atof(latitude) != 0.0 && atof(longitude) != 0.0)
	{
		lati = convertToDecimalDegrees(atof(latitude));							// Store it in variables to send as a payload later
		longi = convertToDecimalDegrees(atof(longitude));
		gpsDataReady = 1;  													// Set flag to indicate valid data
		printf("GPS ready\r\n");
	}
	else
	{
		gpsDataReady = 0;
		printf("GPS not ready\r\n");
	}

	flag = 0;
}

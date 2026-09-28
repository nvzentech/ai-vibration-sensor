/*
 * gps_parser.c
 *
 *  Created on: Mar 22, 2025
 *      Author: 91748
 */

// Includes
#include "main.h"
#include "stdio.h"
#include "gps_parser.h"

#define BUFFER_SIZE 2048
#define SINGLE_PACKET_DATA_SIZE 1024

char uart_rx_buffer[BUFFER_SIZE];
char single_packet_data[SINGLE_PACKET_DATA_SIZE];

// Extract the GNRMC sentence from the UART buffer
char* packet_checking_fun(char* uart_rx_buffer)
{
	int i, k = 0;
	for (i = 0; i < BUFFER_SIZE; i++)
	{
		if (uart_rx_buffer[i] == '$' && uart_rx_buffer[i + 1] == 'G' && uart_rx_buffer[i + 2] == 'N' &&
				uart_rx_buffer[i + 3] == 'R' && uart_rx_buffer[i + 4] == 'M' && uart_rx_buffer[i + 5] == 'C')
		{
			for (; i < BUFFER_SIZE && k < SINGLE_PACKET_DATA_SIZE - 1; i++)
			{
				if (uart_rx_buffer[i] == '*')
				{
					single_packet_data[k++] = uart_rx_buffer[i];
					single_packet_data[k++] = uart_rx_buffer[i + 1]; 	 								// Include the checksum
					single_packet_data[k++] = uart_rx_buffer[i + 2];	 								// Include the checksum
					break;
				}
				single_packet_data[k++] = uart_rx_buffer[i];		     								// Placing the GNMRC data into the buffer --> single_packet_data
			}
			single_packet_data[k] = '\0'; 																// Null-terminate the string
			break; 														 								// Exit after finding the first GNRMC sentence
		}
	}

	return single_packet_data;
}


// Parse the NMEA sentence to extract latitude and longitude
void parse_nmea_sentence(const char *nmea_sentence, char *latitude, char *longitude)
{
	char sentence_copy[SINGLE_PACKET_DATA_SIZE];
	char *token;
	int field_index = 0;
	strncpy(sentence_copy, nmea_sentence, SINGLE_PACKET_DATA_SIZE);										// Copy the NMEA sentence to a local buffer
	token = strtok(sentence_copy, ",");																	// Tokenize the sentence using commas as delimiters

	while (token != NULL)
	{
		field_index++ ;
		if (field_index == 4)																			// Extract latitude (4th field)
		{
			strncpy(latitude, token, 9); 																// Latitude is up to 9 characters (e.g., 4807.038)
			latitude[9] = '\0'; 																		// Null-terminate the string
		}
		if (field_index == 6)																			// Extract longitude (6th field)
		{
			strncpy(longitude, token, 10); 																// Longitude is up to 10 characters (e.g., 01131.000)
			longitude[10] = '\0'; 																		// Null-terminate the string
		}
		token = strtok(NULL, ",");																		// Move to the next token
	}
}

float convertToDecimalDegrees(float nmea_value)
{
	int degrees = (int)(nmea_value / 100);
	float minutes = nmea_value - (degrees * 100);
	return degrees + (minutes / 60);
}




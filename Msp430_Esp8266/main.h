/*
 * main.h
 *
 *  Created on: Sep 15, 2016
 *      Author: nghia
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <msp430.h>
#include <ringBuffer.h>
#include <stdlib.h>

typedef enum
{
	STATE_SEND_RESET_COMMAND  = 0,
	STATE_START_ESP8266,
	STATE_CHECK_AT,
	STATE_PROCESS_ATE0,
	STATE_PROCESS_CWMODE1,
	STATE_PROCESS_DHCP,
	STATE_PROCESS_CONNECT_WIFI,
	STATE_PROCESS_CIPMUX1,
	STATE_PROCESS_CONNECT_IP,
	STATE_SEND_DATA,
	STATE_CLOSE
}systemState;

typedef enum
{
	WIFI_NOT_CONNECT = 0,
	WIFI_CONNECTED,
	WIFI_GOT_IP,
	WIFI_READY
}wifiState;



#endif /* MAIN_H_ */

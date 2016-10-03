//******************************************************************************
//   MSP430G2xx3 Demo - USCI_A0, 115200 UART Echo ISR, DCO SMCLK
//
//   Description: Echo a received character, RX ISR used. Normal mode is LPM0.
//   USCI_A0 RX interrupt triggers TX Echo.
//   Baud rate divider with 1MHz = 1MHz/115200 = ~8.7
//   ACLK = n/a, MCLK = SMCLK = CALxxx_1MHZ = 1MHz
//
//                MSP430G2xx3
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |     P1.2/UCA0TXD|------------>
//            |                 | 115200 - 8N1
//            |     P1.1/UCA0RXD|<------------
//
//   D. Dang
//   Texas Instruments Inc.
//   February 2011
//   Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************
#include "main.h"

uint8_t 			receiveArray[20];					// Array from uart
uint8_t 			indexReceiveArray = 0;				// Index of receive array
uint8_t 			getReceiveString = 0;
uint8_t 			finishSend = 0;
ringBuffer			ringBuff;
uint8_t 			rBufferPop = 0;
uint8_t 			String[20];
//systemState  		sysState = STATE_SEND_RESET_COMMAND;
systemState  		sysState = STATE_START_ESP8266;
wifiState 			wifiStatus = WIFI_NOT_CONNECT;

uint8_t Temp[3];
uint8_t Hum[3];
uint8_t	Cacbon[3];
uint8_t thingspeakString[80];
void SetupUart (void);
void sendTxChar (uint8_t Character);
void sendTxString (uint8_t* String);
void faceData(void);

int main(void)
{
	uint8_t length;
	uint8_t getData = 0;
	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	if (CALBC1_1MHZ==0xFF)					// If calibration constant erased
	{
		while(1);                               // do not load, trap CPU!!
	}
	SetupUart();
	// Delay 10s
	__delay_cycles(160000000);
//	configESP8266();

	while (1)
	{
		// Get data from ringBuffer to String array
		ringBufferPop(&ringBuff,String);
		switch (sysState)
		{
		case STATE_SEND_RESET_COMMAND:
			// Change to other state
			sysState = STATE_START_ESP8266;
			// Reset ESP8266
			sendTxString("AT+RST\r\n");
			break;
		case STATE_START_ESP8266:
			// Check ESP8266 ready to work or not
			//if(strcmp((const char *)String,"ready") == 0)
			//{
				// Change to other state
				sysState = STATE_CHECK_AT;
				// Check AT
				sendTxString("AT\r\n");
			//}
			break;
		case STATE_CHECK_AT:
			// If AT command can work
			if(strcmp((const char *)String,"OK") == 0)
			{
				// Change to other state
				sysState = STATE_PROCESS_ATE0;
				// Disable echo functions
				sendTxString("ATE0\r\n");
			}
			break;
		case STATE_PROCESS_ATE0:
			// If esp8266 can disable echo
			if(strcmp((const char *)String,"OK") == 0)
			{
				// Change to other state
				sysState = STATE_PROCESS_CWMODE1;
				// Change to station Mode
				sendTxString("AT+CWMODE=1\r\n");
			}
			break;
		case STATE_PROCESS_CWMODE1:
			if(strcmp((const char *)String,"OK") == 0)
			{
				// Change to other state
				sysState = STATE_PROCESS_DHCP;
				// Enable DHCP for station mode
				sendTxString("AT+CWDHCP=1,1\r\n");
			}
			break;
		case STATE_PROCESS_DHCP:
			if(strcmp((const char *)String,"OK") == 0)
			{
				// Change to other state
				sysState = STATE_PROCESS_CONNECT_WIFI;
				// Connect to wifi
				sendTxString("AT+CWJAP=\"Vincom-NN\",\"vincom!@!@\"\r\n");
			}
			break;
		case STATE_PROCESS_CONNECT_WIFI:
			if(strcmp((const char *)String,"WIFI CONNECTED") == 0)
			{
				wifiStatus = WIFI_CONNECTED;
			}
			else if(strcmp((const char *)String,"WIFI GOT IP") == 0)
			{
				wifiStatus = WIFI_GOT_IP;
			}
			else if(strcmp((const char *)String,"OK") == 0)
			{
				if (wifiStatus == WIFI_GOT_IP)
				{
					// Change to other state
					sysState = STATE_PROCESS_CIPMUX1;
					wifiStatus = WIFI_READY;
					// Enable multiple connections
					sendTxString("AT+CIPMUX=1\r\n");
				}
			}
			break;
		case STATE_PROCESS_CIPMUX1:
			if(strcmp((const char *)String,"OK") == 0)
			{
				// Change to other state
				sysState = STATE_PROCESS_CONNECT_IP;
				// Connect to ThingSpeak
				sendTxString("AT+CIPSTART=3,\"TCP\",\"184.106.153.149\",80\r\n");
			}
			break;
		case STATE_PROCESS_CONNECT_IP:
			if(strcmp((const char *)String,"ALREAY CONNECT") == 0)
			{
				// Create a face data of sensor
				faceData();
				// Change to other state
				sysState = STATE_SEND_DATA;
				sendTxString("AT+CIPSEND=3,68\r\n");
			}
			else if(strcmp((const char *)String,"3,CLOSED") == 0)
			{
				// Trying to connect to ThingSpeak again
				sendTxString("AT+CIPSTART=3,\"TCP\",\"184.106.153.149\",80\r\n");
			}
			else if(strcmp((const char *)String,"OK") == 0)
			{
				// Create a face data of sensor
				faceData();
				// Change to other state
				sysState = STATE_SEND_DATA;
				sendTxString("AT+CIPSEND=3,68\r\n");
			}
			break;
		case STATE_SEND_DATA:
			if(strcmp((const char *)String,">") == 0)
			{
				// Change to other state
				sysState = STATE_CLOSE;
				sendTxString(thingspeakString);
				sendTxString("\r\n");
			}
			break;
		case STATE_CLOSE:
			if(strcmp((const char *)String,"SEND OK") == 0)
			{
				// delay 4 minutes
				__delay_cycles(0xFFFFFFFF);
				// delay 4 minutes
				__delay_cycles(0xFFFFFFFF);
				// Change to other state
				sysState = STATE_PROCESS_CONNECT_IP;
				// Connect to ThingSpeak
				sendTxString("AT+CIPSTART=3,\"TCP\",\"184.106.153.149\",80\r\n");
			}
			break;
		default:
			break;
		}
	}
}

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
	// Get Rx data to array
	receiveArray[indexReceiveArray++] = UCA0RXBUF;
	// If we receive a <CR><LF> string
	if ((receiveArray[indexReceiveArray - 2]== 0x0D)&&(receiveArray[indexReceiveArray - 1]== 0x0A))
	{
		receiveArray[indexReceiveArray - 2] = 0;
		receiveArray[indexReceiveArray - 1] = 0;
		indexReceiveArray = 0;
//		sendTxString(receiveArray);
//		getReceiveString = 1;
		ringBufferPush(&ringBuff,receiveArray);
	}
	// If we receive a ">"
	else if (receiveArray[indexReceiveArray - 1]== '>')
	{
		receiveArray[indexReceiveArray] = 0;
		indexReceiveArray = 0;
//		getReceiveString = 1;
		ringBufferPush(&ringBuff,receiveArray);
	}
	// If receive string is too Long
	else if (indexReceiveArray >= 20)
	{
		indexReceiveArray = 0;
		return;
	}
}

void SetupUart (void)
{
//	  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
//	  BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
//	  DCOCTL = CALDCO_1MHZ;
		DCOCTL = 0x40;                               // Select lowest DCOx and MODx settings
		BCSCTL1 = CALBC1_16MHZ;                    // Set DCO
		DCOCTL = CALDCO_16MHZ;
	  P1SEL = BIT1 + BIT2 ;                     // P1.1 = RXD, P1.2=TXD
	  P1SEL2 = BIT1 + BIT2;
	  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
//	  UCA0BR0 = 8;                              // 1MHz 115200
	  UCA0BR0 = 138;                              // 1MHz 115200
	  UCA0BR1 = 0;                              // 1MHz 115200
	  UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 5
	  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt

	  __bis_SR_register(GIE);       // Enter LPM0, interrupts enabled
}

void sendTxChar (uint8_t Character)
{
	  while (!(IFG2&UCA0TXIFG));                // USCI_A0 TX buffer ready?
	  UCA0TXBUF = Character;                    // TX -> character
}

void sendTxString (uint8_t* String)
{
	while (*String!=0)
		sendTxChar(*String++);
}

void faceData(void)
{
	strcpy(thingspeakString,"GET /update?api_key=WUZZ3SUHKQUCF89U&field1=");
	Temp[0] 	= 2 + 48;
	Temp[1] 	= rand()%10 + 48;
	Temp[2] 	= '\0';
	Hum[0] 		= (5 + rand()%(4)) + 48;
	Hum[1] 		= rand()%10 + 48;
	Hum[2]		= '\0';
	Cacbon[0] 	= (3 + rand()%(4)) + 48;
	Cacbon[1] 	= rand()%10 + 48;
	Cacbon[2] 	= '\0';
	strcat(thingspeakString,Temp);
	strcat(thingspeakString,"&field2=");
	strcat(thingspeakString,Hum);
	strcat(thingspeakString,"&field3=");
	strcat(thingspeakString,Cacbon);
	strcat(thingspeakString,"\r\n");
}


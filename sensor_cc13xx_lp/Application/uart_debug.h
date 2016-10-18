//////////////////////////////////////////////////////////////////////////////////////////////////////
// 	File name	:	printf.h
// 	Brief 		: 	Support streamming data to the UART port
//	Author 		: 	OS team
//  Note 		: 	
//////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _PRINTF_H_
#define _PRINTF_H_

////////////////////////////////////////// Include Files /////////////////////////////////////////////
#include <stdarg.h>
#include "config.h"
#include "soc_OMAPL138.h"
#include "lcdkOMAPL138.h"

// Include files for UART2
#include "hw_psc_OMAPL138.h"
#include "interrupt.h"
#include "hw_types.h"
#include "uart.h"

///////////////////////////////////// Constant Definitions ///////////////////////////////////////////
#define		PRINT_NUMBER_TYPE			long

#define 	PAD_RIGHT 					1
#define 	PAD_ZERO  					2

#define 	PRINT_BUF_LEN 				12

#define		PRINT_IMMEDIATE_PRINT		1	// 1: Write chars to UART port right in print instruction
											// 0: Don't write char to UART port, get the string only
/////////////////////////////////////// Type Definitions /////////////////////////////////////////////

///////////////////////////// Macros (Inline Functions) Definitions //////////////////////////////////

///////////////////////////////////// Function Prototypes ////////////////////////////////////////////
#if (DEBUG_EN > 0)

void 	            writechar(unsigned char c);
int 		     	print(char **out, const char *format, va_list args );


#else	// Not (DEBUG_EN > 0)

#define 			putchar1(c)											;
#define 			writechar(c)										;
#define 			print(out, format, args )							0

#endif	// (DEBUG_EN > 0)

void                SOS_DEBUG(const char *format,...);
void                initDebugPort(void);

///////////////////////////////////// Variable Definitions ///////////////////////////////////////////


#endif	// _PRINTF_H_

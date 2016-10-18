//////////////////////////////////////////////////////////////////////////////////////////////////////
// 	File name	:	printf.h
// 	Brief 		: 	Support streamming data to the UART port
//	Author 		: 	OS team
//  Note 		: 	
//////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _UART_DEBUG_H_
#define _UART_DEBUG_H_

////////////////////////////////////////// Include Files /////////////////////////////////////////////
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <stdarg.h>
#include "board.h"

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
#if defined(UART_DEBUG)

#define     Board_initGeneral() { \
    Power_init(); \
    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) \
        {System_abort("Error with PIN_init\n"); \
    } \
}

#define     Board_initUART()        UART_init()

void 	            writechar(unsigned char c);
int 		     	print(char **out, const char *format, va_list args );


#else

#define 			putchar1(c)											;
#define 			writechar(c)										;
#define 			print(out, format, args )							0

#endif

void                debug_print(const char *format,...);
void                initDebugPort(void);

///////////////////////////////////// Variable Definitions ///////////////////////////////////////////


#endif	// _UART_DEBUG_H_

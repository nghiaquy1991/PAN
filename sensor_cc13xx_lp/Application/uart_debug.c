//////////////////////////////////////////////////////////////////////////////////////////////////////
// 	File name	:	printf.c
// 	Brief 		: 	Support streamming data to the UART port
//	Author 		: 	OS team.
//  Note 		: 	
//
////////////////////////////////////////// Include Files /////////////////////////////////////////////
#include "debug.h"

#if (DEBUG_EN > 0)

///////////////////////////////////// Constant Definitions ///////////////////////////////////////////

/////////////////////////////////////// Type Definitions /////////////////////////////////////////////

///////////////////////////// Macros (Inline Functions) Definitions //////////////////////////////////

///////////////////////////////////// Function Prototypes ////////////////////////////////////////////

///////////////////////////////////// Variable Definitions ///////////////////////////////////////////

///////////////////////////////////// Function Implements ////////////////////////////////////////////

// ---------------------------------------------------------------------------------------------------
//	Brief	: 	Write a byte to UART port excluding RS485 controlling pins
//		  
// 	Param	: 	Written byte
//  Return	: 	void.
// 	Note	:
// ---------------------------------------------------------------------------------------------------
inline void writechar(unsigned char c)
{
	UARTCharPut(SOC_UART_2_REGS,c);
}

// ---------------------------------------------------------------------------------------------------
//	Brief	: 	Write a char to UART port
//		  
// 	Param	: 	Written char
//  Return	: 	void.
// 	Note	:
// ---------------------------------------------------------------------------------------------------
void printchar(char **str, int c)
{
	if (str) 
	{
		**str = c;
		++(*str);
	}
	else
	{ 
		#if (PRINT_IMMEDIATE_PRINT > 0)
			writechar(c);
		#endif
	}
}

// ---------------------------------------------------------------------------------------------------
//	Brief	: 	Write a string to UART port
//		  
// 	Param	: 	Written string
//  Return	: 	The number of written chars.
// 	Note	:
// ---------------------------------------------------------------------------------------------------
int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) 
	{
		register int len = 0;
		register const char *ptr;

		for (ptr = string; *ptr; ++ptr) 
			++len;

		if (len >= width) 
			width = 0;
		else 
			width -= len;

		if (pad & PAD_ZERO) 
			padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) 
	{
		for ( ; width > 0; --width) 
		{
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) 
	{
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) 
	{
		printchar (out, padchar);
		++pc;
	}

	return pc;
}

// ---------------------------------------------------------------------------------------------------
//	Brief	: 	Write a number to UART port
//		  
// 	Param	: 	Written number
//  Return	: 	The number of written chars.
// 	Note	:
// ---------------------------------------------------------------------------------------------------
int printi(char **out, PRINT_NUMBER_TYPE i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t;
	register int neg = 0, pc = 0;
	register PRINT_NUMBER_TYPE u = i;

	if (i == 0) 
	{
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) 
	{
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) 
	{
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) 
	{
		if( width && (pad & PAD_ZERO) ) 
		{
			printchar (out, '-');
			++pc;
			--width;
		}
		else 
		{
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, pad);
}

// ---------------------------------------------------------------------------------------------------
//	Brief	: 	Write a formated string to UART port
//		  
// 	Param	: 	Written formated string
//  Return	: 	The number of written chars.
// 	Note	:
// ---------------------------------------------------------------------------------------------------
int print(char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) 
	{
		if (*format == '%') 
		{
			++format;
			width = pad = 0;
			if (*format == '\0') 
				break;
			if (*format == '%') 
				goto out;
			if (*format == '-') 
			{
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') 
			{
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) 
			{
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) 
			{
				register char *s = (char *)va_arg( args, int );
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) 
			{
				pc += printi (out, va_arg( args, PRINT_NUMBER_TYPE ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) 
			{
				pc += printi (out, va_arg( args, PRINT_NUMBER_TYPE ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) 
			{
				pc += printi (out, va_arg( args, PRINT_NUMBER_TYPE ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) 
			{
				pc += printi (out, va_arg( args, PRINT_NUMBER_TYPE ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) 
			{
				// char are converted to int then pushed on the stack
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else 
		{
			out:
				printchar (out, *format);
				++pc;
		}
	}
	if (out) 
		**out = '\0';

	va_end( args );	
	
	return pc;
}

// ---------------------------------------------------------------------------------------------------
//	Brief	: 	
//		  
// 	Param	: 	
//  Return	: 	
// 	Note	:
// ---------------------------------------------------------------------------------------------------
int sprintf(char *out, const char *format, ...)
{
	va_list args;

    va_start( args, format );

    return print( &out, format, args );
}

#endif	// (DEBUG_EN > 0)

/*
 * @function: LREP
 * @Brief   : Format the string then write it directly to UART port.
 * @param   : written formated string
 * @return  : void
 */
void SOS_DEBUG (const char *format,...)
{
    va_list args;

    va_start( args, format );
#if DEBUG_EN >0
    print( 0, format, args );
#endif
}

/*
 * @function: initDebugPort
 * @Brief   : init uart port for debugging.
 * @param   : none
 * @return  : none
 * */
void initDebugPort(void)
{
	unsigned int intFlags = 0;
    /* Enabling the PSC for UART2.*/
    PSCModuleControl(SOC_PSC_1_REGS, HW_PSC_UART2, PSC_POWERDOMAIN_ALWAYS_ON,
		     PSC_MDCTL_NEXT_ENABLE);

    /* Setup PINMUX */
    UARTPinMuxSetup(2, FALSE);

    /* Enabling the transmitter and receiver*/
    UARTEnable(SOC_UART_2_REGS);

    /* 1 stopbit, 8-bit character, no parity */
//    config = UART_WORDL_8BITS;

    /* Configuring the UART parameters*/
    UARTConfigSetExpClk(SOC_UART_2_REGS, SOC_UART_2_MODULE_FREQ,
                        BAUD_115200, UART_WORDL_8BITS,
                        UART_OVER_SAMP_RATE_16);

    /* Enabling the FIFO and flushing the Tx and Rx FIFOs.*/
    UARTFIFOEnable(SOC_UART_2_REGS);

    /* Setting the UART Receiver Trigger Level*/
    UARTFIFOLevelSet(SOC_UART_2_REGS, UART_RX_TRIG_LEVEL_1);

    intFlags |= (UART_INT_LINE_STAT  |  \
                 /*UART_INT_TX_EMPTY |    \*/
                 UART_INT_RXDATA_CTI);

    /* Enable the Interrupts in UART.*/
    UARTIntEnable(SOC_UART_2_REGS, intFlags);
}




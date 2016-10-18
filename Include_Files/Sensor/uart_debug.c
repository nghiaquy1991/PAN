//////////////////////////////////////////////////////////////////////////////////////////////////////
// 	File name	:	uart_debug.h
// 	Brief 		: 	Support streamming data to the UART port
//	Author 		: 	OS team.
//  Note 		: 	
//
////////////////////////////////////////// Include Files /////////////////////////////////////////////
#include "uart_debug.h"

UART_Handle 	uart;

#if defined(UART_DEBUG)

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
	UART_write(uart, &c, 1);
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
void debug_print (const char *format,...)
{
    va_list args;

    va_start( args, format );
#if defined(UART_DEBUG)
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
    UART_Params uartParams;
    const char echoPrompt[] = "\fInit debug port success\r\n";

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        System_abort("Error opening the UART");
    }

    UART_write(uart, echoPrompt, sizeof(echoPrompt));
}


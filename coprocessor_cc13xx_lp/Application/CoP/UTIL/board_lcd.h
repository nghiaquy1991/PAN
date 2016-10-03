/******************************************************************************

 @file board_lcd.h

 @brief This file contains the LCD Service definitions and prototypes.

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ti-15.4-stack-sdk_2_00_00_25
 Release Date: 2016-07-14 14:37:14
 *****************************************************************************/
#ifndef BOARD_LCD_H
#define BOARD_LCD_H

/******************************************************************************
 Includes
 *****************************************************************************/

#include <ti/mw/lcd/LCDDogm1286.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 \defgroup BoardLCD Board LCD Functions
 <BR>
 This module is a collection of functions to control the LCD.
 <BR>
 */

/*!
 * \ingroup BoardLCD
 * @{
 */

/*! LCD macros */
#if defined (TI_DRIVERS_LCD_INCLUDED)
/*! Macro definition to write a string to the LCD */
#define LCD_WRITE_STRING(str, line) Board_Lcd_writeString(str, line)
/*! Macro definition to write a string with a value to the LCD */
#define LCD_WRITE_STRING_VALUE(str, value, format, line) \
    Board_Lcd_writeStringValue(str, value, format, line)
#else
/*! Macro definition to write a string to the LCD */
#define LCD_WRITE_STRING(str, line)
/*! Macro definition to write a string with a value to the LCD */
#define LCD_WRITE_STRING_VALUE(str, value, format, line)
#endif

/******************************************************************************
 API Functions
 *****************************************************************************/

#if defined (TI_DRIVERS_LCD_INCLUDED)
/*!
 * @brief   Open LCD peripheral on SRF06EB.
 */
extern void Board_LCD_open(void);
#else
#define Board_LCD_open()
#endif

/*!
 * @brief   Write a string on the LCD display.
 *
 * @param   str - string to print
 * @param   line - line (page) to write (0-7)
 */
extern void Board_Lcd_writeString(char *str, uint8_t line);

/*!
 * @brief   Write a string and value on the LCD display.
 *
 * @param   str - string to print
 * @param   value - value to print
 * @param   format - base of the value to print (2,8,16 etc)
 * @param   line - line (page) to write (0-7)
 */
extern void Board_Lcd_writeStringValue(char *str, uint16_t value,
                                       uint8_t format,
                                       uint8_t line);

/*! @} end group BoardLCD */

#ifdef __cplusplus
}
#endif

#endif /* BOARD_LCD_H */

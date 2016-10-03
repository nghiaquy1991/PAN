/******************************************************************************

 @file board_lcd.c

 @brief This file contains the interface to the LaunchPad LCD Service

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

/******************************************************************************
 Includes
 *****************************************************************************/

#include <string.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/mw/display/Display.h>
#include <ti/mw/display/DisplaySharp.h>
#include <ti/mw/display/DisplayExt.h>

#include "board.h"
#include "board_lcd.h"
#include "util.h"

/******************************************************************************
 Local Constants
 *****************************************************************************/

#define MAX_LCD_BUF 40

/******************************************************************************
 Local Variables
 *****************************************************************************/
#if defined (TI_DRIVERS_LCD_INCLUDED)

/* LCD parameters */
static Display_Params params;
static Display_Handle hDisp;

static DisplaySharpColor_t colors = {.fg = ClrWhite, .bg = ClrBlack};

static uint8_t lcdBuf[MAX_LCD_BUF];

#endif /* TI_DRIVERS_LCD_INCLUDED */

/******************************************************************************
 Public Functions
 *****************************************************************************/

#if defined (TI_DRIVERS_LCD_INCLUDED)
/*!
 Open LCD peripheral on SRF06EB.

 Public function defined in board_lcd.h
 */
void Board_LCD_open(void)
{
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_NONE;

    hDisp = Display_open(Display_Type_LCD, &params);

     Display_control(hDisp, DISPLAYSHARP_CMD_SET_COLORS, &colors );
}
#endif /* TI_DRIVERS_LCD_INCLUDED */

/*!
 * Write a string on the LCD display.
 *
 * Public function defined in board_lcd.h
 */
void Board_Lcd_writeString(char *str, uint8_t line)
{
#if defined (TI_DRIVERS_LCD_INCLUDED)
    if(hDisp != NULL)
    {
        Display_control(hDisp, DISPLAY_CMD_TRANSPORT_OPEN, NULL);
        Display_print0(hDisp, line, 0, str);
        Display_control(hDisp, DISPLAY_CMD_TRANSPORT_CLOSE, NULL);
    }
#endif /* TI_DRIVERS_LCD_INCLUDED */
}

/*!
 * Write a string and value on the LCD display.
 *
 * Public function defined in board_lcd.h
 */
void Board_Lcd_writeStringValue(char *str, uint16_t value, uint8_t format,
                                uint8_t line)
{
#if defined (TI_DRIVERS_LCD_INCLUDED)
    if(hDisp != NULL)
    {
        int len = strlen(str);
        memset(lcdBuf, 0, MAX_LCD_BUF);
        memcpy(lcdBuf, str, len);
        Util_itoa(value, &lcdBuf[len], format);

        Display_control(hDisp, DISPLAY_CMD_TRANSPORT_OPEN, NULL);
        Display_print0(hDisp, line, 0, lcdBuf);
        Display_control(hDisp, DISPLAY_CMD_TRANSPORT_CLOSE, NULL);
    }
#endif /* TI_DRIVERS_LCD_INCLUDED */
}

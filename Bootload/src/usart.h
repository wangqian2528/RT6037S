/**************************************************************************//**
 * @file
 * @brief USART code for the EFM32 bootloader
 * @author Energy Micro AS
* @version 1.63
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2009 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/

#ifndef _UART_H
#define _UART_H

#include <stdint.h>
#include "efm32.h"
#include "compiler.h"
extern unsigned char leuartrx,usartrx;
__ramfunc void USART_printHex(uint32_t integer);
__ramfunc int USART_txByte(uint8_t data);
__ramfunc uint8_t USART_rxByte(void);
__ramfunc void USART_printString(uint8_t *string);
void USART2_init(uint32_t clkdiv);
void LEUART_init(uint32_t clkdiv);
__ramfunc uint8_t LEUART_rxByte(void);
__ramfunc int LEUART_txByte(uint8_t data);
__ramfunc void LEUART_printString(uint8_t *string);
__ramfunc void USART_rxByte1(void);
__ramfunc void LEUART_rxByte1(void);
unsigned char get_Rest_Flag(void);
void Clear_Rest_Flag(void);
extern unsigned char LERXBUF[9];
extern unsigned char UARTRXBUF[9];
#endif

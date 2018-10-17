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

#include "efm32.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "usart.h"
#include "config.h"
#include "em_cmu.h"
#include "em_leuart.h"//2017
/***************************************************************************//**
 * @brief
 *   Prints an int in hex.
 *
 * @param integer
 *   The integer to be printed.
 ******************************************************************************/
 unsigned char REST_Flag;
unsigned char RX_Index;
unsigned char UartRX_Index;
unsigned char LErxdata;
unsigned char LERXBUF[9];
unsigned char UARTRXBUF[9];
__ramfunc void USART_printHex(uint32_t integer)
{
  uint8_t c;
  int     i, digit;
  for (i = 7; i >= 0; i--)
  {
    digit = (integer >> (i * 4)) & 0xf;
    if (digit < 10)
    {
      c = digit + 0x30;
    }
    else
    {
      c = digit + 0x37;
    }
    USART_txByte(c);
  }
}

/***************************************************************************//**
 * @brief
 *   Prints an int in hex.
 *
 * @param integer
 *   The integer to be printed.
 ******************************************************************************/
__ramfunc void LEUART_printHex(uint32_t integer)
{
  uint8_t c;
  int     i, digit;
  for (i = 7; i >= 0; i--)
  {
    digit = (integer >> (i * 4)) & 0xf;
    if (digit < 10)
    {
      c = digit + 0x30;
    }
    else
    {
      c = digit + 0x37;
    }
    LEUART_txByte(c);
  }
}

/**************************************************************************//**
 * @brief Transmit single byte to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc uint8_t USART_rxByte(void)
{
  uint32_t timer = 2000000;
  while (!(BOOTLOADER_USART->STATUS & USART_STATUS_RXDATAV) && --timer ) ;
  if (timer > 0)
  {
    return((uint8_t)(BOOTLOADER_USART->RXDATA & 0xFF));
  }
  else
  {
    return 0;
  }
}
unsigned char rxdata;
__ramfunc void USART_rxByte1(void)
{  uint32_t timer = 20;

  while (!(BOOTLOADER_USART->STATUS & USART_STATUS_RXDATAV) )
    return;
  rxdata=BOOTLOADER_USART->RXDATA;
    if(rxdata=='E')
  	{
   UartRX_Index=0;
   UARTRXBUF[UartRX_Index]=rxdata;
   UartRX_Index++;
    }
  else
  	{
	  	if(rxdata=='T')
		  	{
		  	UARTRXBUF[UartRX_Index] = rxdata;
		    REST_Flag=1;
			}
		  else
		      {
		       
		        UARTRXBUF[UartRX_Index] = rxdata;
		        UartRX_Index++;
		        UartRX_Index %= 9;
		      }
  }
   LEUART_txByte(BOOTLOADER_USART->RXDATA);
   
  //  return((uint8_t)(BOOTLOADER_USART->RXDATA & 0xFF));
 

}

/**************************************************************************//**
 * @brief Transmit single byte to BOOTLOADER_LEUART
 *****************************************************************************/
__ramfunc uint8_t LEUART_rxByte(void)
{
  uint32_t timer = 2000000;
  while (!(BOOTLOADER_LEUART->STATUS & LEUART_STATUS_RXDATAV) && --timer ) ;
  if (timer > 0)
  {
    return((uint8_t)(BOOTLOADER_LEUART->RXDATA & 0xFF));
  }
  else
  {
    return 0;
  }
}

__ramfunc void LEUART_rxByte1(void)
{
 uint32_t timer = 20;
  while (!(BOOTLOADER_LEUART->STATUS & LEUART_STATUS_RXDATAV)  ) 
    return;
  LErxdata=BOOTLOADER_LEUART->RXDATA;
  if(LErxdata==0x7e)
  	{
   RX_Index=0;
   LERXBUF[RX_Index]=LErxdata;
   RX_Index++;
    }
  else
  	{
	  	if(LErxdata==0x0D)
		  	{
		  	LERXBUF[RX_Index] = LErxdata;
		    REST_Flag=1;
			}
		  else
		      {
		       
		        LERXBUF[RX_Index] = LErxdata;
		        RX_Index++;
		        RX_Index %= 9;
		      }
  }
USART_txByte(BOOTLOADER_LEUART->RXDATA);
//           USART_txByte(c);
   // return((uint8_t)(BOOTLOADER_LEUART->RXDATA & 0xFF));

}
unsigned char get_Rest_Flag(void)
{
return REST_Flag;
}

void Clear_Rest_Flag(void)
{
REST_Flag=0;
}
/**************************************************************************//**
 * @brief Transmit single byte to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc int USART_txByte(uint8_t data)
{
  /* Check that transmit buffer is empty */
  while (!(BOOTLOADER_USART->STATUS & USART_STATUS_TXBL)) ;

  BOOTLOADER_USART->TXDATA = (uint32_t) data;
  return (int) data;
}

/**************************************************************************//**
 * @brief Transmit single byte to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc int LEUART_txByte(uint8_t data)
{
  /* Check that transmit buffer is empty */
  while (!(BOOTLOADER_LEUART->STATUS & LEUART_STATUS_TXBL)) ;

  BOOTLOADER_LEUART->TXDATA = (uint32_t) data;
  return (int) data;
}
/**************************************************************************//**
 * @brief Transmit null-terminated string to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc void USART_printString(uint8_t *string)
{
  while (*string != 0)
  {
    USART_txByte(*string++);
  }
}
/**************************************************************************//**
 * @brief Transmit null-terminated string to BOOTLOADER_USART
 *****************************************************************************/
__ramfunc void LEUART_printString(uint8_t *string)
{
  while (*string != 0)
  {
    LEUART_txByte(*string++);
  }
}

/**************************************************************************//**
 * @brief Intializes BOOTLOADER_USART
 *
 * @param clkdiv
 *   The clock divisor to use.
 *****************************************************************************/
void USART_init(uint32_t clkdiv)
{
    //initial 57600  Pc10,Pc11, usart0,#2
    CMU_ClockEnable(cmuClock_UART0,true);
    CMU_ClockEnable(cmuClock_LEUART1,true);
    USART_InitAsync_TypeDef UART_init = USART_INITASYNC_DEFAULT;
    UART_init.baudrate = 115200;
    
    USART_InitAsync(UART0, &UART_init);
    BOOTLOADER_USART->IFC = _USART_IFC_MASK;
    
   // GPIO_PinModeSet(gpioPortC,10,gpioModeInput,1);    //rx
   // GPIO_PinModeSet(gpioPortC,11,gpioModePushPull,1); //tx
   
    GPIO_PinModeSet(gpioPortC,15,gpioModeInput,1);    //rx
    GPIO_PinModeSet(gpioPortC,14,gpioModePushPull,1); //tx
   
    
    BOOTLOADER_USART->ROUTE |=  USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | BOOTLOADER_USART_LOCATION;
    
}

void USART2_init(uint32_t clkdiv)
{
    //initial 57600  Pc10,Pc11, usart0,#2
    CMU_ClockEnable(cmuClock_USART2,true);
    //CMU_ClockEnable(cmuClock_LEUART1,true);
    USART_InitAsync_TypeDef UART_init = USART_INITASYNC_DEFAULT;
    UART_init.baudrate = 115200;
    
    USART_InitAsync(USART2, &UART_init);
    BOOTLOADER_USART->IFC = _USART_IFC_MASK;
   
  USART_IntClear(USART2, USART_IF_RXDATAV);
   // GPIO_PinModeSet(gpioPortC,10,gpioModeInput,1);    //rx
   // GPIO_PinModeSet(gpioPortC,11,gpioModePushPull,1); //tx
   
    GPIO_PinModeSet(gpioPortC,3,gpioModeInput,1);    //rx
    GPIO_PinModeSet(gpioPortC,2,gpioModePushPull,1); //tx
   
    
    BOOTLOADER_USART->ROUTE |=  USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | BOOTLOADER_USART_LOCATION;
  
}
//////////////////////////////////////
void LEUART_init(uint32_t clkdiv)
{
    
    LEUART_TypeDef *leuart = LEUART1;
    //initial 57600  Pc10,Pc11, usart0,#2
  //CMU_ClockEnable(cmuClock_UART0,true);
    CMU_ClockEnable(cmuClock_LEUART1,true);
    LEUART_Init_TypeDef UART_init = LEUART_INIT_DEFAULT;//USART_INITASYNC_DEFAULT;
    UART_init.baudrate = 115200;
    
    //USART_InitAsync(BOOTLOADER_USART, &UART_init);
    LEUART_Init(BOOTLOADER_LEUART, &UART_init);//2017
    //BOOTLOADER_USART->IFC = _USART_IFC_MASK;
      LEUART_IntClear(leuart, LEUART_IF_RXDATAV);
   // GPIO_PinModeSet(gpioPortC,10,gpioModeInput,1);    //rx
   // GPIO_PinModeSet(gpioPortC,11,gpioModePushPull,1); //tx
   
   // GPIO_PinModeSet(gpioPortC,15,gpioModeInput,1);    //rx
   // GPIO_PinModeSet(gpioPortC,14,gpioModePushPull,1); //tx
    GPIO_PinModeSet(gpioPortC,7,gpioModeInput,1);    //rx
    GPIO_PinModeSet(gpioPortC,6,gpioModePushPull,1); //tx
    
    //BOOTLOADER_USART->ROUTE |=  USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | BOOTLOADER_USART_LOCATION;
    leuart->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN|LEUART_ROUTE_LOCATION_LOC0;
    
    
    
    
}
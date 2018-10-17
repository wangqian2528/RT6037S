/**************************************************************************//**
 * @file
 * @brief EFM32 Bootloader. Preinstalled on all new EFM32 devices
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
#include <stdbool.h>
#include "efm32.h"
#include "usart.h"
#include "xmodem.h"
#include "boot.h"
#include "debuglock.h"
#include "autobaud.h"
#include "crc.h"
#include "EFM32_types.h"

#include "config.h"
#include "flash.h"
#include "em_gpio.h"
#include "xmodemleuart.h"
#ifndef NDEBUG
#include "debug.h"
#include <stdio.h>

#endif
/********************标志位宏定义**********************/

#define USER_DATA_BASE          ((uint32_t) 0x0FE00000UL) 
#define SOFT_MAIN_VER_ADDRESS         0
#define SOFT_SECONDARY_VER_ADDRESS    1
#define SETTLE_ADDRESS                2  //按摩完成是否复位 数据为1复位
#define AIRBAG_STRETCH_ADDRESS        3   //按摩椅内部气囊力度
#define SLIDE_MOTOR_ENABLE_ADDRESS    4   //滑动马达使能与禁止
#define PROGRAM_ENABLE_ADDRESS        5   //编程使能地址
#define DEFAULT_TIME_ADDRESS          6   //程序默认时间地址
#define BLUETOOTH_STATUS_ADDRESS      7   //程序蓝牙状态
#define REST_SLEEP_MODE_ADDRESS       8
#define PROGRAM_HAND_ADDRESS       9
#define PROGRAM_FLAG               'p'
/********************标志位宏定义end**********************/


/** Version string, used when the user connects */
#define BOOTLOADER_VERSION_STRING "1.60 "

#pragma location=0x20000000
__no_init uint32_t vectorTable[47];


/*
 * This variable holds the computed CRC-16 of the bootloader and is used during
 * production testing to ensure the correct programming of the bootloader.
 * This can safely be omitted if you are rolling your own bootloader.
 */

#pragma location=0x200000bc
__no_init uint16_t bootloaderCRC;


/* If this flag is set the bootloader will be reset when the RTC expires.
 * This is used when autobaud is started. If there has been no synchronization
 * until the RTC expires the entire bootloader is reset.
 *
 * Essentially, this makes the RTC work as a watchdog timer.
 */
bool resetEFM32onRTCTimeout = false;

/**************************************************************************//**
 * Strings.
 *****************************************************************************/
uint8_t crcString[]     = "\r\nCRC: ";
uint8_t newLineString[] = "\r\n";
uint8_t readyString[]   = "\r\nReady\r\n";
uint8_t okString[]      = "\r\nOK\r\n";
uint8_t failString[]    = "\r\nFail\r\n";
uint8_t unknownString[] = "\r\n?\r\n";
uint8_t jumptoapp[] = "\r\nJump to Application\r\n";
unsigned char HandFlag;//判断手控器升级标志，0x10,打通串口

/**************************************************************************//**
 * @brief RTC IRQ Handler
 *   The RTC is used to keep the power consumption of the bootloader down while
 *   waiting for the pins to settle, or work as a watchdog in the autobaud
 *   sequence.
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt flag */
  RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;
  /* Check if EFM should be reset on timeout */
  if (resetEFM32onRTCTimeout)
  {
#ifndef NDEBUG
    printf("Autobaud Timeout. Resetting EFM32.\r\n");
#endif
    /* Write to the Application Interrupt/Reset Command Register to reset
     * the EFM32. See section 9.3.7 in the reference manual. */
    SCB->AIRCR = 0x05FA0004;
  }
}

/**************************************************************************//**
 * @brief
 *   This function is an infinite loop. It actively waits for one of the
 *   following conditions to be true:
 *   1) The SWDClk Debug pins is not asserted and a valid application is
 *      loaded into flash.
 *      In this case the application is booted.
 *   OR:
 *   2) The SWD Clk pin is asserted and there is an incoming packet
 *      on the USART RX line
 *      In this case we start sensing to measure the baudrate of incoming packets.
 *
 *   If none of these conditions are met, the EFM32G is put to EM2 sleep for
 *   250 ms.
 *****************************************************************************/
void waitForBootOrUSART(void)
{
  uint32_t SWDpins;
#ifndef NDEBUG
  uint32_t oldPins = 0xf;
#endif
  /* Initialize RTC */
  /* Clear interrupt flags */
  RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;
  /* 250 ms wakeup time */
  RTC->COMP0 = (PIN_LOOP_INTERVAL * LFRCO_FREQ) / 1000;
  /* Enable Interrupts on COMP0 */
  RTC->IEN = RTC_IEN_COMP0;
  /* Enable RTC interrupts */
  NVIC_EnableIRQ(RTC_IRQn);
  /* Enable RTC */
  RTC->CTRL = RTC_CTRL_COMP0TOP | RTC_CTRL_DEBUGRUN | RTC_CTRL_EN;

  while (1)
  {
    /* The SWDCLK signal is used to determine if the application
     * Should be booted or if the bootloader should be started
     * SWDCLK (F0) has an internal pull-down and should be pulled high
     *             to enter bootloader mode.
     */
    /* Check input pins */
    SWDpins = GPIO->P[5].DIN & 0x1;

#ifndef NDEBUG
    if (oldPins != SWDpins)
    {
      oldPins = SWDpins;
      printf("New pin: %x \r\n", SWDpins);
    }
#endif

    /* Check if pins are not asserted AND firmware is valid */
    if ((SWDpins != 0x1) && (BOOT_checkFirmwareIsValid()))
    {
      /* Boot application */
#ifndef NDEBUG
      printf("Booting application \r\n");
#endif
      BOOT_boot();
    }

    /* SWDCLK (F0) is pulled high and SWDIO (F1) is pulled low */
    /* Enter bootloader mode */
    if (SWDpins == 0x1)
    {
      /* Increase timeout to 30 seconds */
      RTC->COMP0 = AUTOBAUD_TIMEOUT * LFRCO_FREQ;
      /* If this timeout occurs the EFM32 is rebooted. */
      /* This is done so that the bootloader cannot get stuck in autobaud sequence */
      resetEFM32onRTCTimeout = true;
#ifndef NDEBUG
      printf("Starting autobaud sequence\r\n");
#endif
      return;
    }
    /* Go to EM2 and wait for RTC wakeup. */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __WFI();
  }
}




/**************************************************************************//**
 * @brief
 *   Helper function to print flash write verification using CRC
 * @param start
 *   The start of the block to calculate CRC of.
 * @param end
 *   The end of the block. This byte is not included in the checksum.
 *****************************************************************************/
__ramfunc void verify(uint32_t start, uint32_t end)
{
  USART_printString(crcString);
  USART_printHex(CRC_calc((void *) start, (void *) end));
  USART_printString(newLineString);
}
/********************读取FLASH**********************/

void MEM_Read_Memory(PUINT32 pw_Buffer,int numBytes)
{
  memcpy(pw_Buffer,(uint32_t*)(USER_DATA_BASE),numBytes); 
}

/********************读取FLASH  END**********************/


/********************写FLASH**********************/
void MEM_Write_Memory(PUINT32 pw_Buffer,int numBytes)
{
  MSC_Init();
     __disable_irq();

  MSC_ErasePage((uint32_t*)USER_DATA_BASE);
  
  MSC_WriteWord((uint32_t*)USER_DATA_BASE, pw_Buffer, numBytes);
  
    __enable_irq();
  MSC_Deinit();
}
/********************写FLASH END**********************/
unsigned char exit[]="EXIT";
static void uartPutData(uint8_t * dataPtr, uint32_t dataLen)
{ uint8_t i;
for(i=0;i<dataLen;i++)
{
	USART_txByte(*(dataPtr+i));

}
}


/**************************************************************************//**
 * @brief
 *   The main command line loop. Placed in Ram so that it can still run after
 *   a destructive write operation.
 *   NOTE: __ramfunc is a IAR specific instruction to put code into RAM.
 *   This allows the bootloader to survive a destructive upload.
 *****************************************************************************/
__ramfunc void commandlineLoop(void)
{
  uint32_t flashSize;
  uint8_t  c,d;
  d=0;
  c=0;
   /********************读出FLASH数据保存**********************/
  unsigned int pw_Information[10];
		memset(pw_Information,0,sizeof(pw_Information));
		PBYTE pInformation = (PBYTE)pw_Information;
		MEM_Read_Memory(pw_Information,14);
/********************读出FLASH数据保存END**********************/
  /********************新增两个串口更新程序**********************/
  uint8_t usart_flag,leuart_flag;//判断用那个串口更新驱动板程序
  /********************新增两个串口更新程序END**********************/

  uint8_t *returnString;

  /* Find the size of the flash. DEVINFO->MSIZE is the
   * size in KB so left shift by 10. */
  flashSize = ((DEVINFO->MSIZE & _DEVINFO_MSIZE_FLASH_MASK) >> _DEVINFO_MSIZE_FLASH_SHIFT)
              << 10;
  //flashSize = 0x16000;
  
   flashSize = 0x18000;
  /* The main command loop */
    /********************判断手控器升级标志，0x10,打通串口**********************/
if(HandFlag==0x10)//判断手控器升级标志，0x10,打通串口
{
 while(1)
	 {
	 if(get_Rest_Flag())
	 {
	 if((LERXBUF[1]=='s'&&LERXBUF[2]=='u'&&LERXBUF[3]=='c'&&LERXBUF[4]=='c'&&LERXBUF[5]=='e'&&LERXBUF[6]=='s'))
	{ 

	if(*(pInformation + PROGRAM_HAND_ADDRESS) == 0x10)
		   {
		 *(pInformation + PROGRAM_HAND_ADDRESS) = 0; 
		  MEM_Write_Memory(pw_Information,14);
		   }


	 NVIC_SystemReset(); 
	 	}
	 else if(UARTRXBUF[0]=='E'&&UARTRXBUF[1]=='X'&&UARTRXBUF[2]=='I'&&UARTRXBUF[3]=='T')
	 	{ 
	uartPutData(exit, 4);
	if(*(pInformation + PROGRAM_HAND_ADDRESS) == 0x10)
		   {
		 *(pInformation + PROGRAM_HAND_ADDRESS) = 0; 
		  MEM_Write_Memory(pw_Information,14);
		   }


	 NVIC_SystemReset(); 
	 	}
	 }
	  LEUART_rxByte1();
	  USART_rxByte1();

	 }

}
  /********************判断手控器升级标志，0x10,打通串口end**********************/

  while (1)
  {
     /********************新增两个串口更新程序**********************/
   d= LEUART_rxByte();
    c = USART_rxByte();
    /* Echo */
    if (c != 0)
    {
      leuart_flag=0;
      usart_flag=1;
     // USART_txByte(c);
    }
    if (d != 0)
    {
      leuart_flag=1;
      usart_flag=0;
     // LEUART_txByte(d);
    }
    if(usart_flag==1)
    {
    switch (c)
    {
    /* Upload command */
    case 'u':
      USART_printString(readyString);
      XMODEM_download(BOOTLOADER_SIZE, flashSize);
      break;
    /* Write to user page */
    case 't':
      
      USART_printString(readyString);
      XMODEM_download(XMODEM_USER_PAGE_START, XMODEM_USER_PAGE_END);
      break;
//       case 'S':
//         while(1)
//         {
//               LEUART_rxByte1();
//              USART_rxByte1();        
//         }
//        break;
    /* Write to lock bits */
    case 'p':
      //DEBUGLOCK_startDebugInterface();
      //USART_printString(readyString);
#ifdef USART_OVERLAPS_WITH_BOOTLOADER
      /* Since the UART overlaps, the bit-banging in DEBUGLOCK_startDebugInterface()
       * Will generate some traffic. To avoid interpreting this as UART communication,
       * we need to flush the LEUART data buffers. */
      //BOOTLOADER_USART->CMD = LEUART_CMD_CLEARRX;
#endif
      //XMODEM_download(XMODEM_LOCK_PAGE_START, XMODEM_LOCK_PAGE_END);
      break;
    /* Boot into new program */
    case 'b':
      BOOT_boot();
      break;
    /* Debug lock */
//    case 'l':
//#ifndef NDEBUG
//      /* We check if there is a debug session active in DHCSR. If there is we
//       * abort the locking. This is because we wish to make sure that the debug
//       * lock functionality works without a debugger attatched. */
//      if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0x0)
//      {
//        printf("\r\n\r\n **** WARNING: DEBUG SESSION ACTIVE. NOT LOCKING!  **** \r\n\r\n");
//        USART_printString("Debug active.\r\n");
//      }
//      else
//      {
//        printf("Starting debug lock sequence.\r\n");
//#endif
//      if (DEBUGLOCK_lock())
//      {
//        returnString = okString;
//      }
//      else
//      {
//        returnString = failString;
//      }
//      USART_printString(returnString);
//#ifndef NDEBUG
//        printf("Debug lock word: 0x%x \r\n", *((uint32_t *) DEBUG_LOCK_WORD));
//      }
//#endif
//      break;
//    /* Verify content by calculating CRC of entire flash */
//    case 'v':
//      verify(0, flashSize);
//      break;
//    /* Verify content by calculating CRC of application area */
//    case 'c':
//      verify(BOOTLOADER_SIZE, flashSize);
//      break;
//    /* Verify content by calculating CRC of user page.*/
//    case 'n':
//      verify(XMODEM_USER_PAGE_START, XMODEM_USER_PAGE_END);
//      break;
//    /* Verify content by calculating CRC of lock page */
//    case 'm':
//      verify(XMODEM_LOCK_PAGE_START, XMODEM_LOCK_PAGE_END);
//      break;
    /* Reset command */
    case 'r':
      /* Write to the Application Interrupt/Reset Command Register to reset
       * the EFM32. See section 9.3.7 in the reference manual. */
     // SCB->AIRCR = 0x05FA0004;
      break;
    /* Unknown command */
    case 0:
      /* Timeout waiting for RX - avoid printing the unknown string. */
     // USART_printString(jumptoapp);
    //  BOOT_boot();
      break;
    default:
      USART_printString(unknownString);
    }
    }
    else if(leuart_flag==1)
    {
      switch (d)
    {
    /* Upload command */
    case 'u':
      LEUART_printString(readyString);
      XMODEMLEUART_download(BOOTLOADER_SIZE, flashSize);
      break;
    /* Write to user page */
    case 't':
      
      LEUART_printString(readyString);
      XMODEMLEUART_download(XMODEM_USER_PAGE_START, XMODEM_USER_PAGE_END);
      break;
     
    /* Write to lock bits */
    case 'p':
      //DEBUGLOCK_startDebugInterface();
      //USART_printString(readyString);
#ifdef USART_OVERLAPS_WITH_BOOTLOADER
      /* Since the UART overlaps, the bit-banging in DEBUGLOCK_startDebugInterface()
       * Will generate some traffic. To avoid interpreting this as UART communication,
       * we need to flush the LEUART data buffers. */
      //BOOTLOADER_USART->CMD = LEUART_CMD_CLEARRX;
#endif
      //XMODEM_download(XMODEM_LOCK_PAGE_START, XMODEM_LOCK_PAGE_END);
      break;
    /* Boot into new program */
    case 'b':
      BOOT_boot();
      break;
    /* Debug lock */
//    case 'l':
//#ifndef NDEBUG
//      /* We check if there is a debug session active in DHCSR. If there is we
//       * abort the locking. This is because we wish to make sure that the debug
//       * lock functionality works without a debugger attatched. */
//      if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0x0)
//      {
//        printf("\r\n\r\n **** WARNING: DEBUG SESSION ACTIVE. NOT LOCKING!  **** \r\n\r\n");
//        USART_printString("Debug active.\r\n");
//      }
//      else
//      {
//        printf("Starting debug lock sequence.\r\n");
//#endif
//      if (DEBUGLOCK_lock())
//      {
//        returnString = okString;
//      }
//      else
//      {
//        returnString = failString;
//      }
//      USART_printString(returnString);
//#ifndef NDEBUG
//        printf("Debug lock word: 0x%x \r\n", *((uint32_t *) DEBUG_LOCK_WORD));
//      }
//#endif
//      break;
//    /* Verify content by calculating CRC of entire flash */
//    case 'v':
//      verify(0, flashSize);
//      break;
//    /* Verify content by calculating CRC of application area */
//    case 'c':
//      verify(BOOTLOADER_SIZE, flashSize);
//      break;
//    /* Verify content by calculating CRC of user page.*/
//    case 'n':
//      verify(XMODEM_USER_PAGE_START, XMODEM_USER_PAGE_END);
//      break;
//    /* Verify content by calculating CRC of lock page */
//    case 'm':
//      verify(XMODEM_LOCK_PAGE_START, XMODEM_LOCK_PAGE_END);
//      break;
    /* Reset command */
    case 'r':
      /* Write to the Application Interrupt/Reset Command Register to reset
       * the EFM32. See section 9.3.7 in the reference manual. */
     // SCB->AIRCR = 0x05FA0004;
      break;
    /* Unknown command */
    case 0:
      /* Timeout waiting for RX - avoid printing the unknown string. */
     // USART_printString(jumptoapp);
    //  BOOT_boot();
      break;
    default:
      LEUART_printString(unknownString);
    }
    }
      /********************新增两个串口更新程序end**********************/  
  }
}

/**************************************************************************//**
 * @brief  Create a new vector table in RAM.
 *         We generate it here to conserve space in flash.
 *****************************************************************************/
void generateVectorTable(void)
{
  SCB->VTOR                             = (uint32_t) vectorTable;
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  uint32_t tuning;
  /********************读出FLASH数据保存**********************/
  unsigned int pw_Information[10];
		memset(pw_Information,0,sizeof(pw_Information));
		PBYTE pInformation = (PBYTE)pw_Information;
		MEM_Read_Memory(pw_Information,14);
/********************读出FLASH数据保存END**********************/

  /* Handle potential chip errata */
  /* Uncomment the next line to enable chip erratas for engineering samples */
  /* CHIP_init(); */

  /* Generate a new vector table and place it in RAM */
  generateVectorTable();

  /* Calculate CRC16 for the bootloader itself and the Device Information page. */
  /* This is used for production testing and can safely be omitted in */
  /* your own code. */
  bootloaderCRC  = CRC_calc((void *) 0x0, (void *) BOOTLOADER_SIZE);
  bootloaderCRC |= CRC_calc((void *) 0x0FE081B2, (void *) 0x0FE08200) << 16;
  /* End safe to omit. */

  /* Enable clocks for peripherals. */
  CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;
  CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO | BOOTLOADER_USART_CLOCK;

  /* Enable LE and DMA interface */
  CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE | CMU_HFCORECLKEN0_DMA;

  /* Enable LFRCO for RTC */
  CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;
  /* Setup LFA to use LFRCRO */
  CMU->LFCLKSEL = CMU_LFCLKSEL_LFA_LFRCO | CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2;

  /* Figure out correct flash page size */
  FLASH_CalcPageSize();

  /* Change to 28MHz internal osciallator to increase speed of
   * bootloader */
  tuning = (DEVINFO->HFRCOCAL1 & _DEVINFO_HFRCOCAL1_BAND28_MASK)
           >> _DEVINFO_HFRCOCAL1_BAND28_SHIFT;

  CMU->HFRCOCTRL = CMU_HFRCOCTRL_BAND_28MHZ | tuning;
  /* Wait for the HFRCO to stabilize. This ensures that the period
   * we measure in the autobaud sequence is accurate and not affected
   * by the bootloader being inaccurate. */
  while (!(CMU->STATUS & CMU_STATUS_HFRCORDY)) ;

  /*
    if(GetBlueToothProgram())
    {
      
      
    }
  */
    unsigned char* p;
  unsigned char* H;
    unsigned char programFlag;
	H=(unsigned char*)(XMODEM_USER_PAGE_START+9);
    p = (unsigned char*)(XMODEM_USER_PAGE_START+5);
    programFlag = (unsigned char)*p;
    HandFlag=(unsigned char)*H;
    unsigned char* pSum;
    pSum = (unsigned char*)(CHECKSUM_ADDRESS);
    unsigned short checksum = *pSum;  //获取代码中的checksum
    unsigned char appCheckSum = 0;
    for(unsigned int i=BOOTLOADER_SIZE;i<CHECKSUM_ADDRESS;i++)
    {
      appCheckSum += *(unsigned char*)i;
    }
    if(programFlag == 'p' || checksum != appCheckSum||HandFlag==0X10)
    {
      programFlag=0;
	 
    //  GPIO_PinModeSet(gpioPortF, 4, gpioModePushPull, 1);  //关闭蓝牙
      
      /* Initialize flash for writing */
      FLASH_init();
      
      FLASH_eraseOneBlock(XMODEM_USER_PAGE_START);
      //USART_init(0);
      USART2_init(0);
      LEUART_init(0);
      /* Print a message to show that we are in bootloader mode */
      USART_printString("\r\n\r\n" BOOTLOADER_VERSION_STRING  "ChipID: ");
      /* Print the chip ID. This is useful for production tracking */
      USART_printHex(DEVINFO->UNIQUEH);
      USART_printHex(DEVINFO->UNIQUEL);
      USART_printString("\r\n");
	   /********************清楚工程模式标志位**********************/
//	  if(*(pInformation + PROGRAM_ENABLE_ADDRESS) == PROGRAM_FLAG)
//	  	{
	  	*(pInformation + PROGRAM_ENABLE_ADDRESS) = programFlag; 
	  *(pInformation + PROGRAM_HAND_ADDRESS) = HandFlag; 
       MEM_Write_Memory(pw_Information,14);
//	  	}
	 
	    /********************清楚工程模式标志位 END**********************/
     commandlineLoop();     
    }
    BOOT_boot();
}

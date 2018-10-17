#include "em_gpio.h"
#include "em_usart.h"
#include "Uart0_3D.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#define ARM_BUFFERSIZE          8
#define LEG_BUFFER_LENGTH             9
#include "input.h"
#include "system.h"//20170419

#include "efm32.h"
#include "em_chip.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "string.h"
#include "em_msc.h"
#include "memory.h"
#include "string.h"
static volatile struct circularBuffer
{
    uint8_t  data[ARM_BUFFERSIZE];  /* data buffer */
    // uint32_t rdI;               /* read index */
    uint32_t wrI;               /* write index */
    uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
    bool     overflow;          /* buffer overflow indicator */
}txArmBuf = { 0, 0, 0, false };

volatile unsigned char ucUart0RXBuffer[LEG_BUFFER_LENGTH] = {0};
volatile unsigned char ucLegTXBuffer[LEG_BUFFER_LENGTH] = {0};

unsigned char RX_Leg_Index;
bool  bRXOK_Leg;

unsigned char by_Leg_Key,by_Leg_Key1;

static USART_TypeDef           * uart   = UART0;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;

#define SOI                    0XF0
#define EOI                    0XF1
static bool b485_RX_STATUS=0;
unsigned char  TX_485_Finish;
static bool UartLeg1ms_flag=0;
static unsigned char volatile Uart0errorCount;
static bool b485_Signal_OK=0;
bool  bRXOK_3D=0;
static volatile unsigned char uc3DRXBuffer[LEG_BUFFER_LENGTH] = {0};

static unsigned char massage3DSignal,massage3DPluse,massage3D_Speed;


void UartArm_Initial_IO(void)
{
    /* Enable clock for GPIO module (required for pin configuration) */
    // CMU_ClockEnable(cmuClock_GPIO, true);
    /* Configure GPIO pins */
    GPIO_PinModeSet(UART0_TX_PORT,UART0_TX_BIT,UART0_TX_MODE, 1);//POINT TO 3DCHECK BOAD
    GPIO_PinModeSet(UART0_RX_PORT,UART0_RX_BIT,UART0_RX_MODE, 1);//POINT TO 3DCHECK BOAD
    
    //
    GPIO_PinModeSet(UART0_3085DE_PORT,UART0_3085DE_BIT,UART0_3085DE_MODE, 0);//POINT TO 3DCHECK BOAD
    /* Prepare struct for initializing UART in asynchronous mode*/
    uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
    uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
    uartInit.baudrate     = 115200;//9600;//9600;//         /* Baud rate */
    uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
    uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
    uartInit.parity       = usartNoParity;  /* Parity mode */
    uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
    //uartInit.mvdis        = false;          /* Disable majority voting */
    //uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
    //uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */
    
    /* Initialize USART with uartInit struct */
    USART_InitAsync(uart, &uartInit);
    
    /* Prepare UART Rx and Tx interrupts */
    USART_IntClear(uart, _USART_IF_MASK);
    USART_IntEnable(uart, USART_IF_RXDATAV);
    NVIC_ClearPendingIRQ(UART0_RX_IRQn);
    NVIC_ClearPendingIRQ(UART0_TX_IRQn);
    NVIC_EnableIRQ(UART0_RX_IRQn);
    NVIC_EnableIRQ(UART0_TX_IRQn);
    
    
    /* Enable I/O pins at UART1 location #3 */
    uart->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC0;
    
    /* Enable UART */
    USART_Enable(uart, usartEnable);
    
    
    /* Write welcome message to UART */
    // uartPutData((uint8_t*) welcomeString, welLen);
}
void  UartLeg_init_data(void)
{ 
  b485_RX_STATUS=0;
  TX_485_Finish=0;
  //RX_Leg_Index = 0; 
}


unsigned char UartLeg_TX_STATUS(void)
{
  return TX_485_Finish;
  
}
void UartLeg_1ms_int(void)
{
  
  UartLeg1ms_flag=1;
  
}

unsigned char UartLeg_RX_STATUS(void)
{
  
 return   b485_RX_STATUS;
  
}
bool  Get_b485_Signal_RXStatus(void)
{
  return b485_Signal_OK;
}



void UartLeg_Clr_3DMessageStatus(void)
{
  bRXOK_3D=0;


}

unsigned char UartLeg_Get3DMessage_RXStatus(void)
{
  
    return((unsigned char)bRXOK_3D);
  
}

void  UartLeg_3DMessageCopyData(void)
{
    //memcpy(uc3DRXBuffer,ucUart0RXBuffer,LEG_BUFFER_LENGTH);

    massage3DSignal= ucUart0RXBuffer[3];
    massage3DPluse = ucUart0RXBuffer[4];
    massage3D_Speed=5;
}
unsigned char UartLeg_Get3D_Speed(void)
{
  return(massage3D_Speed);
}
unsigned char UartLeg_Get3DMassageSignal(void)
{
  return(massage3DSignal);//3D机芯开关状态信号
}

unsigned char UartLeg_Get3DPluse(void)
{
  return(massage3DPluse);
}



/**************************************************************************//**
* @brief UART0 RX IRQ Handler
*
* Set up the interrupt prior to use
*
* Note that this function handles overflows in a very simple way.
*
*****************************************************************************/
void UART0_RX_IRQHandler(void)
{   unsigned char chksum,i;
  //  unsigned char by_Data;
    /* Check for RX data valid interrupt */
    
    if (uart->STATUS & USART_STATUS_RXDATAV)
    {
        /* Copy data into RX Buffer */
        uint8_t rxData = USART_Rx(uart);
        if((rxData != SOI)&&(RX_Leg_Index==0))return;//20170419
        if((rxData == SOI)&&(RX_Leg_Index==0))
        {
            //RX_Leg_Index = 0;  
            ucUart0RXBuffer[RX_Leg_Index] = rxData;
            RX_Leg_Index++;
            b485_RX_STATUS=1;//20170419
            //b485_RX_Timeout=0;//20170419
        }
        else
        {
          /*
            if(rxData == EOI)
            {     
                 
                ucUart0RXBuffer[RX_Leg_Index] = rxData;
                bRXOK_Leg = 1; 
            }
            else
            {
                ucUart0RXBuffer[RX_Leg_Index] = rxData;
                RX_Leg_Index++;
                //RX_Leg_Index %= LEG_BUFFER_LENGTH;
            }
            */
            ucUart0RXBuffer[RX_Leg_Index]=rxData;
            RX_Leg_Index++;
            RX_Leg_Index %= LEG_BUFFER_LENGTH;
            if( ucUart0RXBuffer[1]==0x01)// TO LEG
            {
            }
            if( ucUart0RXBuffer[1]==0x02)// TO 3D CHEC
            {
              if(rxData==EOI)
              {
                chksum=0;
                for(i=1;i<3+ucUart0RXBuffer[2];i++)
                {
                  
                  chksum+=ucUart0RXBuffer[i];
                }
                chksum  = ~chksum;
                chksum  &=   0x7f;
                if(chksum==ucUart0RXBuffer[3+ucUart0RXBuffer[2]])
                {  // b485_RX_STATUS=0;
                  //t2++;
                  Uart0errorCount=0;
                  RX_Leg_Index=0;
                  bRXOK_3D = 1;    
                  b485_Signal_OK=1;
                  
                }
              }//if(rxData==EOI)
            } //if( ucUart0RXBuffer[1]==0x02)// TO 3D CHEC   
            
            
            
            
            
        }//ELSE
        
        /* Clear RXDATAV interrupt */
        USART_IntClear(UART0, USART_IF_RXDATAV);
    }
}

/**************************************************************************//**
* @brief UART1 TX IRQ Handler
*
* Set up the interrupt prior to use
*
*****************************************************************************/
void UART0_TX_IRQHandler(void)
{
    uint32_t irqFlags = USART_IntGet(UART0);
    
    /* Check TX buffer level status */
    if (uart->STATUS & USART_STATUS_TXBL)
    {
        if (txArmBuf.pendingBytes > 0)
        {
            TX_485_Finish=1;//20170419
            /* Transmit pending character */
            USART_Tx(uart, txArmBuf.data[txArmBuf.wrI]);
            txArmBuf.wrI++;
            txArmBuf.pendingBytes--;
        }
        
        /* Disable Tx interrupt if no more bytes in queue */
        if (txArmBuf.pendingBytes == 0)
        {
            USART_IntDisable(uart, USART_IF_TXBL);
            //UartARM_3085DE_Off();//
              TX_485_Finish=0;
              UartLeg1ms_flag=0;
              System_clr_Counter();
            
            
            
        }
    }
}

void uart0PutData(uint8_t * dataPtr, uint32_t dataLen)
{
  int i = 0;

  /* Check if buffer is large enough for data */
  if (dataLen > ARM_BUFFERSIZE)
  {
    /* Buffer can never fit the requested amount of data */
    return;
  }
  /*
  if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE)
  {
    while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) ;
  }
  */
  while (i < dataLen)
  {
    txArmBuf.wrI = 0;
    txArmBuf.data[i] = *(dataPtr + i);
    i++;
  }

  /* Increment pending byte counter */
  txArmBuf.pendingBytes = dataLen;

  /* Enable interrupt on USART TX Buffer*/
  //UartARM_3085DE_On();
  USART_IntEnable(uart, USART_IF_TXBL);
}

void Uart0_Transmit_Packet(unsigned char* buf,unsigned int length)
{
  
  UartARM_3085DE_On();
  TX_485_Finish=1;
  uart0PutData(buf,length);
}

unsigned char UartLeg_GetRXStatus(void)
{
  return((unsigned char)bRXOK_Leg);
}

void UartLeg_ClearRXStatus(void)
{
  bRXOK_Leg = 0;
}

void UartARM_3085DE_On(void)
{
    GPIO_PinOutSet(UART0_3085DE_PORT,UART0_3085DE_BIT);   
}

void UartARM_3085DE_Off(void)
{
    GPIO_PinOutClear(UART0_3085DE_PORT,UART0_3085DE_BIT);
}
void UartLeg_TX_RX_STATUS(void)
{
  

  
 if( TX_485_Finish)
 {
  // UartLeg_SET_TX_EN();
   
 }
  else
  {
    
   if(UartLeg1ms_flag)
   {
   UartARM_3085DE_Off();
   }
   
    
  }
  
}



void UartLeg_10msInt(void)
{// b485_RX_STATUS=0;
  if(Uart0errorCount < 255) Uart0errorCount++;
  if(Uart0errorCount >30) b485_Signal_OK = false;  //10ms*30=300ms
}


/*


void UartLeg_RX_TimeoutInt(void)//5ms接受时间
{
  
 if( b485_RX_Timeout<255)b485_RX_Timeout++;
 
 if(b485_RX_Timeout>=5)b485_RX_STATUS=0;
  
  
}
*/
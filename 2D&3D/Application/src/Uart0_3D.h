#ifndef __UART_LEG_H__
#define __UART_LEG_H__

#define UART0_TX_PORT           gpioPortF
#define UART0_TX_BIT            6
#define UART0_TX_MODE           gpioModePushPull

#define UART0_RX_PORT           gpioPortF
#define UART0_RX_BIT            7
#define UART0_RX_MODE           gpioModeInputPull

#define UART0_3085DE_PORT           gpioPortF
#define UART0_3085DE_BIT            5
#define UART0_3085DE_MODE           gpioModePushPull

void UartArm_Initial_IO(void);
void Uart0_Transmit_Packet(unsigned char* buf,unsigned int length);
unsigned char UartLeg_GetRXStatus(void);
void UartLeg_ClearRXStatus(void);
//unsigned short UartLeg_GetAngle(void);
//unsigned char UartLeg_GetFlexStatus(void);
//unsigned char UartLeg_GetLegStatus(void);
void UartARM_3085DE_On(void);
void UartARM_3085DE_Off(void);
//unsigned int Input_GetArmStatus(void);

void UartLeg_1ms_int(void);

void UartLeg_10msInt(void);
bool  Get_b485_Signal_RXStatus(void);

void UartLeg_Clr_3DMessageStatus(void);
unsigned char UartLeg_Get3DMessage_RXStatus(void);


unsigned char UartLeg_Get3DMassageSignal(void);
void UartLeg_TX_RX_STATUS(void);
void  UartLeg_3DMessageCopyData (void);
void  UartLeg_init_data(void);
unsigned char UartLeg_Get3DPluse(void);




#endif
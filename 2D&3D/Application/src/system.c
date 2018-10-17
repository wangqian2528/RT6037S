//#include "efm32.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "EFM32_def.h"
#include "EFM32_types.h"
#include "em_letimer.h"
#include "em_vcmp.h"
#include "system.h"
#include "Timer.h"
#include "LegMotor.h"
#include "BackPad.h"
#include "WalkMotor.h"
#include "SlideMotor.h"
#include "em_gpio.h"
#include "input.h"
#include "Valve.h"
#include "Data_Cul.h"
#include "IndicateLED.h"
#include "ADC_Scan.h"
#include "LED_RGB.h"        
#include "KneadMotor.h"        
#include "Walkmotor.h"        
#include "AxisMotor.h"        
#include "em_wdog.h"
#include "HandUart_New.h"
#include "signalUart_new.h"
#include "VoiceUart.h"
#include "WaistHot.h"
#include "Flex_Uart.h"
#include "Uart0_3D.h"//#include "UartLeg.h"
extern void main_30ms_int(void);
extern void main_50ms_int(void);
extern void main_10ms_int(void);
extern void main_100ms_int(void);

extern void main_5ms_int(void);
extern void main_200ms_int(void);
bool  bTimer2MS;
unsigned int sysCounter=0;

unsigned char n3Dtime;
unsigned int n3Dtimecount;


//extern unsigned long by_moni_cmd_tm;

void LETIMER_setup(void)
{
    /* Enable necessary clocks */
    // CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    // CMU_ClockEnable(cmuClock_CORELE, true);
    // CMU_ClockEnable(cmuClock_LETIMER0, true);  
    // CMU_ClockEnable(cmuClock_GPIO, true);
    
    /* Configure PD6 and PD7 as push pull so the
    LETIMER can override them */
   
    /* Set initial compare values for COMP0 and COMP1 
    COMP1 keeps it's value and is used as TOP value
    for the LETIMER.
    COMP1 gets decremented through the program execution
    to generate a different PWM duty cycle */
    LETIMER_CompareSet(LETIMER0, 0, 32768);
    //  LETIMER_CompareSet(LETIMER0, 1, 5000);
    
    /* Repetition values must be nonzero so that the outputs
    return switch between idle and active state */
    //  LETIMER_RepeatSet(LETIMER0, 0, 0x01);
    //  LETIMER_RepeatSet(LETIMER0, 1, 0x01);
    
    /* Route LETIMER to location 0 (PD6 and PD7) and enable outputs */
    //LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_OUT1PEN | LETIMER_ROUTE_LOCATION_LOC0;
    
    /* Set configurations for LETIMER 0 */
    const LETIMER_Init_TypeDef letimerInit = 
    {
        .enable         = true,                   /* Start counting when init completed. */
        .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
        .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
        .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
        .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
        .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
        .out0Pol        = 0,                      /* Idle value for output 0. */
        .out1Pol        = 0,                      /* Idle value for output 1. */
        .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
        .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
        .repMode        = letimerRepeatFree       /* Count until stopped */
    };
    
    /* Initialize LETIMER */
    LETIMER_Init(LETIMER0, &letimerInit); 
}

void System_Initial_IO(void)
{
    CHIP_Init();
    CMU_HFRCOBandSet(cmuHFRCOBand_28MHz);  /** main system clock - internal RC 28MHz*/
    SystemCoreClockUpdate();
    CMU_ClockEnable(cmuClock_HFPER,true);  /** High frequency peripheral clock */
    CMU_ClockEnable(cmuClock_CORELE, true);/* Enable CORELE clock */
    CMU_ClockEnable(cmuClock_GPIO,true);   /** General purpose input/output clock. */
    CMU_ClockEnable(cmuClock_USART0,true); //165和valve
    CMU_ClockEnable(cmuClock_USART1,true); //与小腿通讯接口
    CMU_ClockEnable(cmuClock_USART2,true); //wifi接口
    CMU_ClockEnable(cmuClock_UART0,true);  //手控器接口
    CMU_ClockEnable(cmuClock_UART1,true);  //语音模块接口
    CMU_ClockEnable(cmuClock_LEUART1,true);  //机芯接口
    CMU_ClockEnable(cmuClock_ADC0, true);  
    CMU_ClockEnable(cmuClock_TIMER0,true); 
    CMU_ClockEnable(cmuClock_TIMER1,true);
    CMU_ClockEnable(cmuClock_TIMER2,true);
    CMU_ClockEnable(cmuClock_TIMER3,true);
    //CMU_ClockEnable(cmuClock_PRS, true);
    CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */
    //CMU_ClockEnable(cmuClock_LEUART0, true);    /* Enable LEUART1 clock */
    SysTick_Config(SystemCoreClock / 1000); //set 1ms interupt using systick
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO); //LFA选择内部32768时钟
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFRCO);
    //GPIO_PinModeSet(TEST_PORT, TEST_BIT, TEST_MODE, 1);
    CMU_ClockEnable(cmuClock_LETIMER0, true);  
    LETIMER_setup();
    /* Enable underflow interrupt */  
    LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);  
    /* Enable LETIMER0 interrupt vector in NVIC*/
    NVIC_EnableIRQ(LETIMER0_IRQn);
    
    CMU_IntClear(CMU_IFC_LFXORDY);
    CMU_IntEnable(CMU_IEN_LFXORDY);
    NVIC_EnableIRQ(CMU_IRQn);
    CMU_OscillatorEnable(cmuOsc_LFXO,1,0);
    
    VCMP_Init_TypeDef vcmp =
    {
        true,                               /* Half bias current */
        0,                                  /* Bias current configuration */
        true,                               /* Enable interrupt for falling edge */
        false,                              /* Enable interrupt for rising edge */
        vcmpWarmTime256Cycles,              /* Warm-up time in clock cycles */
        vcmpHyst20mV,                       /* Hysteresis configuration */
        1,                                  /* Inactive comparator output value */
        false,                              /* Enable low power mode */
        VCMP_VoltageToLevel(2.5), /* Trigger level */
        false                               /* Enable VCMP after configuration */
    };
    /* Initialize VCMP */
    CMU_ClockEnable(cmuClock_VCMP, true);
    VCMP_Init(&vcmp);
    
    /* Enable VCMP interrupt lines */
    NVIC_EnableIRQ(VCMP_IRQn);
    VCMP_IntEnable(VCMP_IEN_EDGE | VCMP_IEN_WARMUP);
    
    /* Enable VCMP and wait for warm-up complete */
    VCMP_Enable();  
}
//1ms interupt

bool bFlag1ms;
void SysTick_Handler(void)
{
    static BYTE by_Time5ms = 0;
    static BYTE by_Time10ms = 0;
    static BYTE by_Time50ms = 0;
    static BYTE by_Time100ms = 0;
    static BYTE by_Time200ms = 0;

    bFlag1ms = 1;
  //  sysCounter++;
    if(sysCounter>=2)
    {
        UartLeg_1ms_int();
        sysCounter=0;
    }
    else
    {
      sysCounter++;
    }
    
  //  Valve_1ms_Int();
 //   VoiceUart_1msInt();
    Input_Back_Pulse1MS();
       
 // UartLeg_1ms_int();//不可以放在这里，数据传输会漏帧

  //  UartLeg_RX_TimeoutInt();
    
    if(by_Time5ms >= 4)
    { 
      
  by_Time5ms=0;
      
        Valve_5ms_Int();
        Input_5ms_Int();
        Axis_5ms_Int();
      //  main_5ms_int();
        
        
    }                          
    else ++by_Time5ms;
    if(by_Time10ms >= 9)
    { 
        by_Time10ms = 0;  
        WDOG_Feed();

        SlideMotor_10ms_Int();
        BackMotor_10ms_int();
        LegMotor_10ms_int();
        KneadMotor_10ms_Int();
        main_10ms_int();
        LED_RGB_10ms_Int();
  
        SignalBoard_10msInt();//读取电机行程 开关计数
        AxisMotor_10msInt();
        WalkMotor_10msInt();
        Problem_10ms_Int();
        KnockMotor_10msInt();
        Valve_10ms_Int();
         UartLeg_10msInt();
        FlexMotor_10ms_Int();//add20170427
    }                          
    else ++by_Time10ms;
    
	if(n3Dtimecount>1000)
			{
			n3Dtimecount=0;
			n3Dtime++;
		}
		++n3Dtimecount;

    
    if(by_Time50ms >= 50)
    {                          
        by_Time50ms = 0;         
        main_50ms_int();

    }                          
    else ++by_Time50ms;
    
    if(by_Time100ms >= 100)
    {                          
        by_Time100ms = 0;  
        LED_RGB_100ms_Int();
        WaistHeat_100ms_Int();
     //   VoiceUart_100msInt();

        AxisMotor_100msInt();
        Timer_Flag_100ms_Int();
        FlexMotor_100ms_Int();
	
        
        main_100ms_int();//100ms上传手控器一次 数据给
        
	 //if(by_moni_cmd_tm>1)by_moni_cmd_tm--;   // 减到1不减。
	
	
    }                          
    else ++by_Time100ms;
  if(by_Time200ms >= 200)
  {                          
    by_Time200ms = 0;
    main_200ms_int();
  }                          
  else ++by_Time200ms;
}

void System_Delay_us(uint32_t ulData)
{
    while(ulData > 0)
    {
      __NOP();__NOP();__NOP();__NOP();__NOP();
      __NOP();__NOP();__NOP();__NOP();__NOP();
      __NOP();__NOP();__NOP();__NOP();__NOP();
      __NOP();__NOP();__NOP();__NOP();__NOP();
      ulData--;
    }
}

void CMU_IRQHandler(void)
{
    NVIC_DisableIRQ(CMU_IRQn);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);  //选择外部32768时钟
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);  //选择外部32768时钟
    CMU_OscillatorEnable(cmuOsc_LFRCO,0,0);  //禁止内部32768时钟，省电 
}
/**************************************************************************//**
* @brief LETIMER0_IRQHandler
* Interrupt Service Routine for LETIMER
* 中断时间1秒钟
*****************************************************************************/
void LETIMER0_IRQHandler(void)
{ 
    /* Clear LETIMER0 underflow interrupt flag */
    LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
    Data_Flag_Int();

    // IndicateLED_Toggle();
}


unsigned int  System_GetCounter(void)
{ 
    return(sysCounter);
}

void System_clr_Counter(void)
{
  
  sysCounter=0;
}
/***************************************************************************//**
* @brief
*   VCMP interrupt handler, triggers on EDGE and WARMUP events
******************************************************************************/
extern unsigned int password;
void VCMP_IRQHandler()
{
    /* Execute on WARMUP interrupt */
    if (VCMP->IF & VCMP_IF_WARMUP)
    {
        /* Enable Low Power Reference */
        // VCMP_LowPowerRefSet(true);
        
        /* Clear interrupt flag */
        VCMP_IntClear(VCMP_IFC_WARMUP);
    }
    
    /* Execute on EDGE interrupt */
    if (VCMP->IF & VCMP_IF_EDGE)
    {
        /* Low voltage warning */
        password = 0;  //低电压清除password
        
        /* Clear interrupt flag */
        VCMP_IntClear(VCMP_IFC_EDGE);
    }
}


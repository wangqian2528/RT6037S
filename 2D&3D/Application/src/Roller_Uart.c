#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "Roller_Uart.h"
#include "ADC_Scan.h"

unsigned int displayPWM;

static unsigned char nRollerSpeed;
void Roller_SetSpeed(unsigned char rollerSpeed)
{
   nRollerSpeed = rollerSpeed;
  
}
unsigned char Roller_GetSpeed(void)
{
	unsigned char level = 0;
	 if(nRollerSpeed == ROLLER_SPEED_SLOW) level = 1;
	 if(nRollerSpeed == ROLLER_SPEED_MID) level = 2;
	 if(nRollerSpeed == ROLLER_SPEED_FAST) level = 3;
	 return level;

}

unsigned char Roller_GetSpeed_old(void)
{

    return nRollerSpeed;
}


//static bool bFalg; 
void Rooler_Initial_IO(void)
{
  GPIO_PinModeSet(ROLLER_MOTOR_RESET_PORT, ROLLER_MOTOR_RESET_BIT, ROLLER_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(ROLLER_MOTOR_ENBL_PORT, ROLLER_MOTOR_ENBL_BIT, ROLLER_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(ROLLER_MOTOR_PHASE_PORT, ROLLER_MOTOR_PHASE_BIT, ROLLER_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(ROLLER_MOTOR_DECAY_PORT, ROLLER_MOTOR_DECAY_BIT, ROLLER_MOTOR_DECAY_MODE, 0);
  GPIO_PinModeSet(ROLLER_MOTOR_FAULT_PORT, ROLLER_MOTOR_FAULT_BIT, ROLLER_MOTOR_FAULT_MODE, 1);
  
  TIMER_InitCC_TypeDef timerCCInit = ROLLER_MOTOR_Timer_CCInit;
  /* Configure CC channel 0 */
  TIMER_InitCC(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL, &timerCCInit);
  
  /* Set Top Value */
  TIMER_TopSet(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_DEFAULT_TOP);
  
  TIMER_CompareBufSet(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL, 0);
  
  TIMER_Init_TypeDef timerInit = ROLLER_MOTOR_Timer_Init;
  /* Configure timer */
  TIMER_Init(ROLLER_MOTOR_TIMER, &timerInit);
  
  /* Route CC0 to location 3 (PD1) and enable pin */  
  ROLLER_MOTOR_TIMER->ROUTE |= (ROLLER_MOTOR_ROUTE_EN | ROLLER_MOTOR_ROUTE_LOCATION); 
}
//void RollerMotor_10ms_Int()
//{
 // bFalg = true;
//}
unsigned int RollerMotor_VoltageAdj(unsigned int setDuty)
{
  unsigned short adc24;      //此处的电压值已经扩大了100倍
  ADC_Get_Voltage(ADC_V24,&adc24);     
  if(adc24 <= ROLLER_SET_VOLTAGE/100) 
  {
    return setDuty;        //电压值偏低，返回预设值
  }
  unsigned int scale = ROLLER_SET_VOLTAGE / adc24; //计算与设定电压的比例值
  setDuty *= scale;
  unsigned int yushu = setDuty  % 100;
  setDuty /= 100;
  if(yushu > 50) setDuty++;
  return setDuty; 
}

void RollerMotor_Set_Pwm_Data(unsigned long ulDuty)
{
  if(ulDuty == 0)
  {
    TIMER_CompareBufSet(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL, ulDuty);
    return;
  }
  ulDuty = RollerMotor_VoltageAdj(ulDuty);
  if(RollerMotor_Get_Fault() == ROLLER_MOTOR_FAIL)
  {
    RollerMotor_Reset();
    __no_operation();
    __no_operation();
    RollerMotor_Reset_Cancel();
  }
  TIMER_CompareBufSet(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL, ulDuty);
}

unsigned long RollerMotor_Get_Pwm_Data(void)
{
  return(TIMER_CompareBufGet(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL));
}

bool RollerMotor_IsRun(void)
{
  unsigned long  ulDuty;
  ulDuty = TIMER_CompareBufGet(ROLLER_MOTOR_TIMER, ROLLER_MOTOR_TIMER_CHANNEL);
  if(ulDuty > 0) return 1;
  else return 0;
}
void RollerMotor_Set_Current(int current)
{
}

void RollerMotor_UnClockRun(void)
{
  GPIO_PinOutSet(ROLLER_MOTOR_RESET_PORT, ROLLER_MOTOR_RESET_BIT);
  GPIO_PinOutClear(ROLLER_MOTOR_PHASE_PORT, ROLLER_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(ROLLER_MOTOR_DECAY_PORT, ROLLER_MOTOR_DECAY_BIT);
}
void RollerMotor_ClockRun(void)
{
  GPIO_PinOutSet(ROLLER_MOTOR_RESET_PORT, ROLLER_MOTOR_RESET_BIT);
  GPIO_PinOutSet(ROLLER_MOTOR_PHASE_PORT, ROLLER_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(ROLLER_MOTOR_DECAY_PORT, ROLLER_MOTOR_DECAY_BIT);
}
void RollerMotor_Break(void)
{
  RollerMotor_Set_Pwm_Data(0);
  GPIO_PinOutClear(ROLLER_MOTOR_DECAY_PORT, ROLLER_MOTOR_DECAY_BIT);
  //GPIO_PinOutClear(ROLLER_MOTOR_ENBL_PORT, ROLLER_MOTOR_ENBL_BIT);
}

void RollerMotor_Control(unsigned int ulDuty,unsigned int phase)
{
    Roller_SetSpeed(ulDuty);
  RollerMotor_Set_Pwm_Data(ulDuty);
  displayPWM = ulDuty;
  if(phase)
  {
    RollerMotor_ClockRun();
  }
  else
  {
   RollerMotor_UnClockRun();
  }
}

void RollerMotor_Reset(void)
{
  GPIO_PinOutClear(ROLLER_MOTOR_RESET_PORT, ROLLER_MOTOR_RESET_BIT);
  RollerMotor_Set_Pwm_Data(0);
}
void RollerMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(ROLLER_MOTOR_RESET_PORT, ROLLER_MOTOR_RESET_BIT);
}

int RollerMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(ROLLER_MOTOR_FAULT_PORT, ROLLER_MOTOR_FAULT_BIT))
    return ROLLER_MOTOR_NORMAL;
  return ROLLER_MOTOR_FAIL;
}


void RollerMotor_Controlstrtch(unsigned int speed,unsigned int phase)
{
  //Roller_SetSpeed(speed);
   // Roller_SetMode(phase);
    Roller_SetSpeed(speed);
  RollerMotor_Set_Pwm_Data(speed);
  //displayPWM = speed;
  if(phase)
  {
    RollerMotor_ClockRun();
  }
  else
  {
   RollerMotor_UnClockRun();
  }
}

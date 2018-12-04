#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "LegMotor.h"
#include "Flex_Uart.h"
#include "input.h"
#include "EFM32_def.h"
#include "timer.h"
static bool bFalg ; 
#include "BackPad.h"
unsigned int Flex_In_Fault;
//static unsigned char nFlexMode;
//static unsigned char nFlexDirection;
//static unsigned char nFlexPower;
//static unsigned char nFlexCurrent;
static unsigned int Flex_In_Delay,nFlex_In_TimeStop,nFlex_StretchIn_TimeStop;
static bool b10msFlexinFalg,b100msFalg;
extern bool bHaveMan;
//static bool FlexMotorEnable = false;
//角度禁止标志 当角度大于45度时，不允许伸出，以防顶翻机器
//static bool angleFlexDisable = true;
static unsigned int nFlexMotorRunStateOld = STATE_FLEX_IDLE ;
static unsigned int w_FlexAdjStep;
//static unsigned char nFlexStatus;
static bool FlexMotorEnable = false;
//static unsigned char nFlexMotorInFlag, nFlexMotorInFlag1 ; //1: runin ; 0:run out or idle
BYTE w_Timer_OutTime;
static volatile unsigned int w_Timer;
void FlexMotor_Initial_IO(void)
{
  GPIO_PinModeSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT, FLEX_MOTOR_RESET_MODE, 0);
  GPIO_PinModeSet(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT, FLEX_MOTOR_ENBL_MODE, 0);
  GPIO_PinModeSet(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT, FLEX_MOTOR_PHASE_MODE, 0);
  GPIO_PinModeSet(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT, FLEX_MOTOR_DECAY_MODE, 0);
 // GPIO_PinModeSet(FLEX_MOTOR_CURRENT_PORT, FLEX_MOTOR_CURRENT_BIT, FLEX_MOTOR_CURRENT_MODE, 1);
  GPIO_PinModeSet(FLEX_MOTOR_FAULT_PORT, FLEX_MOTOR_FAULT_BIT, FLEX_MOTOR_FAULT_MODE, 1);
  
  TIMER_InitCC_TypeDef timerCCInit = FLEX_MOTOR_Timer_CCInit;
  
  //FLEX_MOTOR
  TIMER_InitCC(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, &timerCCInit);
  //TIMER_InitCC(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, &timerCCInit);
  
  TIMER_TopSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_DEFAULT_TOP);
  
  //FLEX_MOTOR
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, 0);
  
  TIMER_Init_TypeDef timerInit = FLEX_MOTOR_Timer_Init;

  TIMER_Init(FLEX_MOTOR_TIMER, &timerInit);
  FLEX_MOTOR_TIMER->ROUTE |= (FLEX_MOTOR_ROUTE_EN | FLEX_MOTOR_ROUTE_LOCATION); 
 // FLEX_MOTOR_TIMER->ROUTE |= (FLEX_MOTOR_ROUTE_EN | FLEX_MOTOR_ROUTE_CUR_EN | \
  //  ROLLER_MOTOR_ROUTE_EN | FLEX_MOTOR_ROUTE_LOCATION); 
  
  //TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, FLEX_CURRENT_4A); //最大电流4A(80/131*3.3)
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 // TIMER_InitCC_TypeDef timerCCcurInit = FLEX_MOTOR_CUR_Timer_CCInit;
  /* Configure CC channel 0 */
//  TIMER_InitCC(FLEX_MOTOR_CUR_TIMER, FLEX_MOTOR_CUR_TIMER_CHANNEL, &timerCCcurInit);
  
  /* Set Top Value */
//  TIMER_TopSet(FLEX_MOTOR_CUR_TIMER, FLEX_MOTOR_CUR_DEFAULT_TOP);
  
//  TIMER_CompareBufSet(FLEX_MOTOR_CUR_TIMER, FLEX_MOTOR_CUR_TIMER_CHANNEL, FLEX_CURRENT_4A);
  
//  TIMER_Init_TypeDef timercurInit = FLEX_MOTOR_CUR_Timer_Init;
  /* Configure timer */
//  TIMER_Init(FLEX_MOTOR_CUR_TIMER, &timercurInit);
  
  /* Route CC0 to location 3 (PD1) and enable pin */  
//  FLEX_MOTOR_CUR_TIMER->ROUTE |= (FLEX_MOTOR_CUR_ROUTE_EN | FLEX_MOTOR_CUR_ROUTE_LOCATION); 
}

/*
void FlexMotor_Data_Init(void)
{
  //nFlexMotorInFlag = 0 ;
  //nFlexMotorInFlag1 = 0 ;
}
*/
void FlexMotor_10ms_Int(void)
{
  bFalg = true; 
   b100msFalg = true; 
    b10msFlexinFalg = true;
}
void FlexkMotor_Set_Pwm_Data(unsigned long ulDuty)
{
  unsigned int duty ;
  if(ulDuty == 0)
  {
    TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, ulDuty);
    //EMC
    GPIO_PinOutClear(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT); 
    return;
  }
  duty = TIMER_CompareBufGet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL);
  /*if(ulDuty == duty)
  {
   if(FlexMotor_Get_Fault() == FLEX_MOTOR_NORMAL) return;
    FlexMotor_Reset();
    __no_operation();
    __no_operation();
    FlexMotor_Reset_Cancel();
   return; 
  }
  */

   if(FlexMotor_Get_Fault() == FLEX_MOTOR_FAIL) 
   {
      FlexMotor_Reset();
      __no_operation();
      __no_operation();
      FlexMotor_Reset_Cancel();
   }

  if(!bFalg) return;
  bFalg = false;
  if(duty < ulDuty)//小于设定值，则取最大值的一半后 ，加++
  {
    if(duty < FLEX_MOTOR_DEFAULT_TOP/2)
      duty = FLEX_MOTOR_DEFAULT_TOP/2;
    else duty++;
  }
  else  //大于，就直接赋值
  {
    duty = ulDuty;
  }
  TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL, duty);
}

int FlexPower_On(unsigned char speed)
{
  FlexkMotor_Set_Pwm_Data(speed);
  /*
  int val = 0;
  Power_On();
  if(GPIO_PinOutGet(FLEX_MOTOR_ENBL_PORT,FLEX_MOTOR_ENBL_BIT))
  { //端口已经high
    if(FlexMotor_Get_Fault() == FLEX_MOTOR_FAIL)
    {
      FlexMotor_Reset();
      __no_operation();
      __no_operation();
      FlexMotor_Reset_Cancel();
      __no_operation();
      if(FlexMotor_Get_Fault() != FLEX_MOTOR_FAIL) val = 0;
    }
  }
  else
  {
   GPIO_PinOutSet(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
   val =  1;
  }
  */
  return 0;
}

void FlexPower_Off(void)
{
  //GPIO_PinOutClear(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
  FlexkMotor_Set_Pwm_Data(0);
}

unsigned int FlexPower_Get(void)
{
  unsigned long  ulDuty;
  ulDuty = TIMER_CompareBufGet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CHANNEL);
  if(ulDuty > 0) return FLEX_POWER_ON;
  else return FLEX_POWER_OFF;
}

void FlexMotor_Out(void)
{
 // Power_On();
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
  //GPIO_PinOutSet(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);  //20170112
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
  //140531
  //nFlexMotorInFlag = 0 ;
  
}

void FlexMotor_In(void)
{
 // Power_On();
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
  //GPIO_PinOutClear(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);
  GPIO_PinOutSet(FLEX_MOTOR_PHASE_PORT, FLEX_MOTOR_PHASE_BIT);
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);  
  //140531
  //nFlexMotorInFlag = 1 ;
  //140603
  //nFlexMotorInFlag1 = 1 ;
}

void FlexMotor_Break(void)
{
  GPIO_PinOutClear(FLEX_MOTOR_DECAY_PORT, FLEX_MOTOR_DECAY_BIT);
  //140531
  //nFlexMotorInFlag = 0 ;
 // GPIO_PinOutClear(FLEX_MOTOR_ENBL_PORT, FLEX_MOTOR_ENBL_BIT);
  FlexkMotor_Set_Pwm_Data(0);
}

void FlexMotor_Reset(void)
{
  GPIO_PinOutClear(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
}

void FlexMotor_Reset_Cancel(void)
{
  GPIO_PinOutSet(FLEX_MOTOR_RESET_PORT, FLEX_MOTOR_RESET_BIT);
}

int FlexMotor_Get_Fault(void)
{
  if(GPIO_PinInGet(FLEX_MOTOR_FAULT_PORT, FLEX_MOTOR_FAULT_BIT))
    return FLEX_MOTOR_NORMAL;
  return FLEX_MOTOR_FAIL;
}

//FlexPad motor control function
//正常运行中返回0，遇到行程开关或其它原因停止返回1
  bool bLegPressFlag;// Input_Get_Leg_Press_Status();
unsigned char FlexMotor_Control(unsigned char nFinalFlexPadMotorState,unsigned char speed,unsigned char current)
{
  static unsigned int position = 0;
  unsigned char nRetVal ;
 // unsigned char nResetSteps, nResetSteps1;
 // bool bLegPressFlag;// Input_Get_Leg_Press_Status();
  bool bPowerFlag;
  nRetVal = FLEX_RUN;
//  nResetSteps = 0 ;//normal reset
//  nResetSteps1 = 0 ;//auto flexin
  //current = FLEX_CURRENT_2A;  //temp test
  
  bLegPressFlag = Input_Get_Leg_Press_Status();
  
  //140603
  if(nFlexMotorRunStateOld != nFinalFlexPadMotorState)
  {
    nFlexMotorRunStateOld = nFinalFlexPadMotorState ;
    position = 0 ;
  }
  
  switch(nFinalFlexPadMotorState)
  {
    /*
  case STATE_RUN_FLEX_AUTO:
    {
    if(Input_GetFlexFootSwitch() == FOOT_SWITCH_ON)
    {  //碰到脚了
    bPowerFlag = FALSE;
    nRetVal = FLEX_STOP_AT_FOOT ;
    FlexMotor_Break();
    break;
  }
    else
    {
  }
  }
    break;
    */
  case STATE_RUN_FLEX_IN://自动向里
    

    if( (Input_GetFlexInSwitch() == REACH_FLEX_LIMIT) || (position == 1)||(bLegPressFlag==0))
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_IN ;
      FlexMotor_Break();
      break;
    }
    //向里找到脚，在向脚心移动1秒，停
    if((Input_GetFlexFootSwitch() == FOOT_SWITCH_ON)&&(b10msFlexinFalg == true) )
    {
      Flex_In_Delay++;       //碰到脚，计数延时  fww
      b10msFlexinFalg = false;
    }
    if((Input_GetFlexFootSwitch() == FOOT_SWITCH_ON)&&(Flex_In_Delay >= 100) )
    {  //碰到脚了
      Flex_In_Delay = 0;
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_FOOT ;
      FlexMotor_Break();
      break;
    }
    if(nFlex_In_TimeStop  < 2000)
    {
      
      if(b100msFalg == true)
      {
        b100msFalg = false;
        nFlex_In_TimeStop++;
      }
      if(nFlex_In_TimeStop >=2000)
      {
        Flex_In_Delay = 0;
        bPowerFlag = FALSE;
        nRetVal = FLEX_STOP_AT_FOOT ;
        FlexMotor_Break();
        break;
      }
    }
    //140531
    //Clear_Accident_flag() ;
    position = 0;
    bPowerFlag = TRUE;
    FlexMotor_In();
    
    break ;
  case STATE_RUN_FLEX_RESET: //强制收回
    

    if(Input_GetFlexInSwitch() == REACH_FLEX_LIMIT || (position == 1)||(bLegPressFlag==0))
    {
      position = 1;
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_IN ;
      FlexMotor_Break();
      break;
    }
    //碰到脚了 ，就停
    /* if((Input_GetFlexFootSwitch() == FOOT_SWITCH_ON) )
    {  //碰到脚了
    bPowerFlag = FALSE;
    nRetVal = FLEX_STOP_AT_FOOT ;
    FlexMotor_Break();
    break;
  }
    */
    
    
    position = 0;
    bPowerFlag = TRUE;
    FlexMotor_In();
    
    break ;
  case STATE_RUN_FLEX_STRETCH_RESET:   
        if(Input_GetFlexInSwitch() == REACH_FLEX_LIMIT || position == 1)
        {
            position = 1;
            bPowerFlag = FALSE;
            nRetVal = FLEX_STOP_AT_IN ;
            FlexMotor_Break();
            break;
        }
        if(nFlex_StretchIn_TimeStop  < 1300)
        {
            if(b100msFalg == true)
            {
                b100msFalg = false;
                nFlex_StretchIn_TimeStop++;
            }
            if(nFlex_StretchIn_TimeStop >=1300)
            {
                Flex_In_Delay = 0;
                bPowerFlag = FALSE;
                nRetVal = FLEX_STOP_AT_FOOT ;
                FlexMotor_Break();
                break;
            }
        }
        position = 0;
        bPowerFlag = TRUE;
        FlexMotor_In();
        break ;   
  case STATE_RUN_FLEX_OUT:  //上行  自动向外
    if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
    {
      //140531
      //Clear_Accident_flag() ;
      
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_OUT ;
      break;
    }
    if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON)
    { //小于15度
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_ANGLE ;
      break;
    }
    if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)
    { //碰到地面了
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_GROUND;
      break;
    }
    /*if(Input_GetFlexFootSwitch() == FOOT_SWITCH_OFF)
    {  //碰不到脚了
    bPowerFlag = FALSE;
    nRetVal = FLEX_STOP_AT_FOOT_LEAVE ;
    FlexMotor_Break();
    break;
  }*/
    if(Input_GetFlexFootSwitch() == FOOT_SWITCH_ON)
    {  //碰到脚了
      //bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_FOOT ;
      //FlexMotor_Break();
      //break;
    } 
    if((Input_GetFlexFootSwitch() == FOOT_SWITCH_OFF)&&((w_Timer > w_Timer_OutTime)))
    {  //碰不到脚了
      bPowerFlag = FALSE;
      nRetVal = FLEX_STOP_AT_FOOT_LEAVE ;
      FlexMotor_Break();
      break;
    }
    
    
    //140603
    //Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;nFlex_In_TimeStop = 0;
    break ;
    
case STATE_RUN_FLEX_STRETCH_OUT:  //向外运行，如果原来有脚向外运行到找不到脚停止，如果原来没脚向外运行2秒停止
        if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
        {
            __NOP();
            position = 2;
            bPowerFlag = FALSE;
            FlexMotor_Break();
            nRetVal = FLEX_STOP_AT_OUT ;
            break;
        }
        if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON)//小于15度
        { 
            bPowerFlag = FALSE;
            FlexMotor_Break();
            nRetVal = FLEX_STOP_AT_ANGLE ;
            break;
        }
        if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)//碰到地面了
        { 
            bPowerFlag = FALSE;
            FlexMotor_Break();
            nRetVal = FLEX_STOP_AT_GROUND;
            break;
        }
        if(Input_GetFlexFootSwitch() == FOOT_SWITCH_ON)//碰到脚了
        {  
            nRetVal = FLEX_STOP_AT_FOOT ;
        } 
        if(bHaveMan == TRUE)
        {
            if((Input_GetFlexFootSwitch() == FOOT_SWITCH_OFF))//碰不到脚了
            {  
                bPowerFlag = FALSE;
                nRetVal = FLEX_STOP_AT_FOOT_LEAVE ;
                FlexMotor_Break();
                break;
            }
        }
        else
        {
            if(w_Timer > 20)
            { 
                bPowerFlag = FALSE;
                nRetVal = 0x04 ;
                FlexMotor_Break();
                bPowerFlag = false;
                break;
            }
        }
        nFlex_In_TimeStop = 0;
        position = 0;
        FlexMotor_Out();
        bPowerFlag = TRUE;
        break ;
  case STATE_RUN_FLEX_MANUAL_OUT:  //小腿手动伸出
    if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT )//|| position == 2)
    {
      nRetVal = FLEX_STOP_AT_OUT ;
      //Clear_Accident_flag() ;
      
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nFlex_In_TimeStop = 0;
      break;
    }
    if( position == 2)
    {
      nRetVal = FLEX_STOP_AT_OUT ;
      //Clear_Accident_flag() ;
      
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nFlex_In_TimeStop = 0;
      break;
    }
    if(Input_GetFlexGroundSwitch() == LEGGROUND_SWITCH_ON)
    { //碰到地面了
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_GROUND;
      break;
    }
    if(Input_GetFlexAngleSwitch() == LEGANGLE_SWITCH_ON)
    { //小于15度
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_ANGLE ;
      break;
    }
    Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;  
  case STATE_RUN_FLEX_TEST_OUT:  //小腿伸出直到碰到行程开关，测试用
    if(Input_GetFlexOutSwitch() == REACH_FLEX_LIMIT || position == 2)
    {
      position = 2;
      bPowerFlag = FALSE;
      FlexMotor_Break();
      nRetVal = FLEX_STOP_AT_OUT ;
      break;
    }
    position = 0;
    FlexMotor_Out();
    bPowerFlag = TRUE;
    break ;    
  case STATE_FLEX_IDLE:
    //140603
    Timer_Counter_Clear(FLEX_TIME_CHANNEL) ;
    //140526
    position = 0;
    bPowerFlag = FALSE;
    FlexMotor_Break();
    nRetVal = FLEX_STOP_AT_IDLE;nFlex_In_TimeStop = 0;
    break ;
  default://异常处理
    break ;
  }
  //电源部分的处理
  
  
  
  if(bPowerFlag == TRUE)
  {
    //FlexPower_On(current);
    FlexPower_On(speed);
    //TIMER_CompareBufSet(FLEX_MOTOR_TIMER, FLEX_MOTOR_TIMER_CUR_CHANNEL, current); 
    // TIMER_CompareBufSet(FLEX_MOTOR_CUR_TIMER, FLEX_MOTOR_CUR_TIMER_CHANNEL, current);
    // TIMER_CompareBufSet(FLEX_MOTOR_CUR_TIMER, FLEX_MOTOR_CUR_TIMER_CHANNEL, current); 
  }
  else
  {
    FlexPower_Off();
  }
  return nRetVal ;
}
//自动找脚程序 

void FlexMotorFollowingFood(void)
{
  BYTE FlexMotor_ReturnVal;
  if(!FlexMotorEnable)
  {
    Flex_In_Delay = 0;//fww
    Flex_In_Fault = 0;
    nFlex_In_TimeStop = 0;
    return;
  }
  if(LegPower_Get())
  {
    //141125
   // ucPowerOff = FALSE;
    Flex_In_Delay = 0;//fww
    Flex_In_Fault = 0;//fww
    FlexMotor_Break();//fww
    return;  //小腿上下电动缸还在运行中
  }
  if(BackMotor_GetPower() == 0)
  {
    //141125
   // ucPowerOff = FALSE;
    Flex_In_Delay = 0;//fww
    Flex_In_Fault = 0;//fww
    FlexMotor_Break();//fww
    return;  //小腿上下电动缸还在运行中
  }
  if((FlexMotor_Get_Fault()==FLEX_MOTOR_FAIL) ||(Flex_In_Fault>=120)) //fww
  {
    FlexMotorEnable = false;                    //fww
    FlexMotor_Break();                     //fww
    return;
  }
  
  switch(w_FlexAdjStep)
  {
  case 0:
    //判断马达位置（极限位 或 不是极限位）
    //极限位，伸出变量为6秒， 不是极限位伸出变量为2秒
    if(  Input_GetFlexInSwitch() == REACH_FLEX_LIMIT  )
    {
      w_Timer_OutTime = 30;
    }
    else
    {
      w_Timer_OutTime = 10;
    }

   //运行下一步
    w_FlexAdjStep = 1;
    Flex_In_Fault = 0;
    w_Timer = 0;    
    
    /*
    if(Input_GetFlexFootSwitch() == FOOT_SWITCH_ON)
    {  
      w_FlexAdjStep = 1;
    }
    else
    {  
      w_FlexAdjStep = 2;
    } 
    Flex_In_Fault = 0;
    */
    break;
  case 1://先向外走
   // if(FlexMotor_Control(STATE_RUN_FLEX_OUT,FLEX_SPEED_FAST,FLEX_CURRENT_3A))
   // {
   //   w_FlexAdjStep++;
   // }
    FlexMotor_ReturnVal = FlexMotor_Control(STATE_RUN_FLEX_OUT,FLEX_SPEED_FAST,FLEX_CURRENT_3A);
    if (  (  (FlexMotor_ReturnVal&FLEX_STOP_AT_OUT) == FLEX_STOP_AT_OUT)
        ||(  (FlexMotor_ReturnVal&FLEX_STOP_AT_ANGLE) == FLEX_STOP_AT_ANGLE)
          ||(  (FlexMotor_ReturnVal&FLEX_STOP_AT_GROUND) == FLEX_STOP_AT_GROUND)
            ||(  (FlexMotor_ReturnVal&FLEX_STOP_AT_FOOT_LEAVE) == FLEX_STOP_AT_FOOT_LEAVE))
    {
      w_FlexAdjStep++;
      w_Timer = 0;
    }
    if ( (FlexMotor_ReturnVal&FLEX_STOP_AT_FOOT) == FLEX_STOP_AT_FOOT)
    {
        w_Timer_OutTime = 10;
        w_Timer = 0;
    }  
    Flex_In_Delay = 0;
    Flex_In_Fault = 0;
    break;
  case 2://再向里走
    if(FlexMotor_Control(STATE_RUN_FLEX_IN,FLEX_SPEED_FAST,FLEX_CURRENT_2A))
    {
      FlexMotorEnable = false;
    }
    break;
  }
}
//1 执行自动跟脚程序
int FlexMotorGetEnable(void)
{
  return(FlexMotorEnable);
}
void FlexMotorSetEnable(void)
{
  FlexMotorEnable = true;
  w_FlexAdjStep = 0; 
}
void FlexMotorSetDisable(void)
{
  FlexMotorEnable = false;
  w_FlexAdjStep = 0; 
}

/*
*@brief   :
*@param   :
*@retval  :
******************************************/
void FlexMotor_100ms_Int(void)
{
  //Flex_100ms_Flag = true;
  w_Timer++;
}
void nSet_StretchTime( BYTE time)
{
  w_Timer = time;
  
}
BYTE nGet_StretchTime( void)
{
  return (w_Timer );
  
}

void nSet_StretchStopTime(unsigned int time)
{
  nFlex_StretchIn_TimeStop = time;
}






/*
*@brief   :
*@param   :
*@retval  :
******************************************/
//void Clear_Accident_flag(void)
//{
//  Accident_Happen_Flag = 0 ;
//}
/*
*@brief   :
*@param   :
*@retval  :
******************************************/
//void Clear_Accident1_flag(void)
//{
//  Accident_Happen_Flag1 = 0 ;
//}
/*
*@brief   :
*@param   :
*@retval  :
******************************************/
//unsigned char Get_CurAccident_flag(void)
//{
//  return Accident_Happen_Flag ;
//}

//http://www.doc88.com/p-0941988996630.html    http://blog.csdn.net/efm32/article/details/12564491
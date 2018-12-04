
/*
* Function: Input signal process
*/
/************************************************************************************/

#include "efm32.h"
#include "em_chip.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "efm32_types.h"
#include "efm32_def.h"
#include "LegMotor.h"
#include "Valve.h"
#include "WalkMotor.h"
#include "AxisMotor.h"
#include "MassageStatus.h"
#include "memory.h"
#include "signalUart_new.h"
#include "backaction.h"
#include "BackPad.h"

#include "Uart0_3D.h"//#include "UartLeg.h"

#include "input.h"

BITS InputData1;
#define bLegDownSwitch 	        InputData1.bD0   
#define bLegUpSwitch		InputData1.bD1   
#define bSlidePadUpSwitch	InputData1.bD2  
#define bSlidePadDownSwitch	InputData1.bD3   
#define bWalkDownSwitch 	InputData1.bD4   
#define bWalkUpSwitch           InputData1.bD5   
#define bBackUpSwitch		InputData1.bD6
#define bBackDownSwitch		InputData1.bD7

#define Data1Offset         0
#define Data2Offset         1
#define Data3Offset         2
BITS InputData2;
#define bFootSwitch     InputData2.bD3
#define bAngleSwitch    InputData2.bD4
#define bGroudSwitch    InputData2.bD5
#define bFlexInSwitch   InputData2.bD6
#define bFlexOutSwitch  InputData2.bD7
typedef  struct
{
        unsigned char timer_H;
        unsigned char timer_L;
        unsigned char flag;
}INPUT_ST;
//bool bVout;
bool bVshoulder=0;
bool vReadSholuder=0;
bool bWalkChange,bInputReady;
bool _3DfrontSwitch, _3DBackSwitch;
  
INPUT_ST st_FlexGround,st_FlexOut,st_FlexIn,st_Foot,st_Angle,st_Vout,st_AxisSW;
INPUT_ST st_SlideUp,st_SlideDown,st_BackUp,st_BackDown,st_LegUp,st_LegDown,st_WalkUp,st_WalkDown;
bool b5msFlag;
BYTE nPulseHigh;
UINT16 WalkCount = 0xffff;  // 0xffff indicate no initial, at top =0 at bottom = maximal, up:dec down:add
BYTE by_KneadPosition = KNEAD_WIDTH_UNKNOWN;
volatile unsigned short nCurWalkLocate;
//unsigned short nCounterCurWalkLocate;
unsigned short nCurAxisLocate;
unsigned short nCounterCurAxisLocate;
unsigned int tickAxisCount;
bool bKneadMin,bKneadMid,bKneadMax,bBalance;
bool bWalkPulseCount,bPreWalkPulseCount;
//unsigned int axisTickTimedec[100];
//unsigned int axisTickTimeadd[100];


volatile unsigned int nCurBackLocate;

bool bShould,bShouldDelay;

unsigned int tet,tet2;
unsigned int cont;

unsigned int phawe;
/***************************


添加肩部检测到位后，再延时继续走几秒钟，才是真正的肩部位置
******************************/

/**************************************************************************//**
 * @brief GPIO_ODD_IRQHandler
 * Interrupt Service Routine Odd GPIO Interrupt Line

  if(GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT))//PHASE=1 电机向上，坐标计数减小
  {
    if(w_Position > 0) w_Position--;  
  }
  else
  {
    if(w_Position < BACK_MOTOR_MAX_POSITION) w_Position++;  
  }
 *****************************************************************************/
//get backmotor's pulse
#define BACK_PULSE_CHECK_TIME 3
unsigned char BackPulseCountHigh = 0;
unsigned char BackPulseCountLow = 0;
unsigned char BackPulseStep = 0;

unsigned short test_count=0;
void Input_Back_Pulse1MS(void)
{
  bool flag = GPIO_PinInGet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT);
  if(flag == 0){
    //低电平为开关有效
    BackPulseCountLow++;
  }else{
    BackPulseCountHigh++;
  }
  //从高到底再到高为1个有效数
  switch(BackPulseStep){
  case 0:
    if((flag == 1) && (BackPulseCountHigh > BACK_PULSE_CHECK_TIME)){
      BackPulseCountLow = 0;
      BackPulseStep++;
    }
    break;
  case 1:
    //从高到底再到高为1个有效数
    if((flag == 0) && (BackPulseCountLow > BACK_PULSE_CHECK_TIME)){
      BackPulseCountHigh = 0;
      BackPulseStep = 0;
      //计数
      if(!GPIO_PinOutGet(BACK_MOTOR_PHASE_PORT, BACK_MOTOR_PHASE_BIT) != 0)
      {
        if(nCurBackLocate > 0 )nCurBackLocate-- ;
		//test_count--;
		
      }
	  else
	  {
        nCurBackLocate++ ;
		//test_count++;
		
      }
    }
    break;
  }
}







void GPIO_ODD_IRQHandler(void)
{
  
}

void GPIO_EVEN_IRQHandler(void)
{ 

  if(GPIO_IntGet()&(1<<INPUT_WALK_PULSE_BIT))//读取3D机芯的行走电机脉冲信号
  {
    bWalkChange = TRUE;
    GPIO_IntClear(1<<INPUT_WALK_PULSE_BIT);
    if(WalkMotor_GetDirection() != WALK_MOTOR_GO_UP)
    {
      if(nCurWalkLocate > 0 )
      {
          nCurWalkLocate-- ;
      }
    }
    else
    {
      if(nCurWalkLocate < 1000)
      {
          nCurWalkLocate++ ;
      }
    }
  } 
}

void Input_SetWalkMotorPosition(unsigned short locate)
{
//#ifdef RT8600S
      nCurWalkLocate = (unsigned short)locate<<2;
//#else
  
 //     nCurWalkLocate = (unsigned short)locate*2;
//#endif
  
}
unsigned short Input_GetWalkMotorPosition(void)//
{

       return(nCurWalkLocate>>2);//4分频

}

//相反方向
void Input_SetCounterWalkMotorPosition(unsigned short locate) //8600
{
  return;
  //nCounterCurWalkLocate = (unsigned short)locate;
  //nCounterCurWalkLocate *= 2;
}
/*
unsigned short Input_GetCounterWalkMotorPosition(void) //8600
{
  return(nCounterCurWalkLocate/2);
}
*/
void Input_SetAxisMotorPosition(unsigned short locate)//3D机芯镜向坐标
{
  nCurAxisLocate = (unsigned short)locate;
}
unsigned short Input_GetAxisMotorPosition(void)
{
  return(nCurAxisLocate);
}

//相反方向
void Input_SetCounterAxisMotorPosition(unsigned short locate) //8600
{
  nCounterCurAxisLocate = (unsigned short)locate;
  nCounterCurAxisLocate *= 2;
}
unsigned short Input_GetCounterAxisMotorPosition(void) //8600
{
  return(nCounterCurAxisLocate/2);
}

void Input_Initial_IO(void)
{
 
  GPIO_PinModeSet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, INPUT_WALK_PULSE_MODE, 1);
 // GPIO_IntConfig(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT, false, true, true);
  
  GPIO_PinModeSet(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, INPUT_BACK_PULSE_MODE, 1);
//  GPIO_IntConfig(INPUT_BACK_PULSE_PORT, INPUT_BACK_PULSE_BIT, false, true, true);
  
  GPIO_PinModeSet(LEG_PRESS_PORT, LEG_PRESS_BIT, LEG_PRESS_MODE, 1); 
  
}
void Input_5ms_Int(void)
{
    tickAxisCount++;
    if(AxisMotor_IsRun())
    {
      tickAxisCount++;
    }
    b5msFlag = 1;
}

void Input_High(INPUT_ST* p)
{
  p->timer_L = 0;
  p->timer_H ++; 
  if((p->timer_H) >= 2)
  {
    p->timer_H = 2;
    p->flag = 1;
  }
}
void Input_Low(INPUT_ST* p)
{
  p->timer_H = 0;
  p->timer_L ++; 
  if((p->timer_L) >= 2)
  {
    p->timer_L = 2;
    p->flag = 0;
  }
}
/*

BITS InputData1;
#define bLegDownSwitch 	        InputData1.bD0   
#define bLegUpSwitch		InputData1.bD1   
#define bSlidePadUpSwitch	InputData1.bD2  
#define bSlidePadDownSwitch	InputData1.bD3   
#define bWalkDownSwitch 	InputData1.bD4   
#define bWalkUpSwitch           InputData1.bD5   
#define bBackUpSwitch		InputData1.bD6
#define bBackDownSwitch		InputData1.bD7
*/


void Input_Proce(void)
{
	__no_operation();
		  __no_operation();

  static volatile unsigned char SignalBoard_First_signal,signal,SignalBoard_Second_signal,SignalBoard_Three_signal;
  static volatile unsigned char M3Dsignal;

  static int counter;  
  static unsigned short nOldCurAxisLocate = 0;
  if(!b5msFlag) return;		
  b5msFlag = 0;
  if(++counter >= 4) bInputReady = true;
  /*
  if(UartLeg_Get3DMessage_RXStatus())
  {
    UartLeg_Clr_3DMessageStatus();
    UartLeg_3DMessageCopyData ();
    
    
  }
  */

  nCurAxisLocate = UartLeg_Get3DPluse();//3D机芯镜向位置坐标
  if(nOldCurAxisLocate |= nCurAxisLocate);
  {
    AxisMotor_UpdataPosition();
    nOldCurAxisLocate = nCurAxisLocate;
  }
  /*
  massage3DSignal= ucUart0RXBuffer[3];
  massage3DPluse = ucUart0RXBuffer[4];
  massage3D_Speed=5;
  */
    M3Dsignal = UartLeg_Get3DMassageSignal();//读取3D行程开关信号
  //		   7        6	     5              4	           3            2   1   0//敲击平位置=3,       4=肩膀位置
  //xxxx xxxx  --> 0 3DbackSwitch 3DfrontSwitch shoulderPosition MotorPosition Max Med Min
  
  if(M3Dsignal&BIT2)
  {
    by_KneadPosition = KNEAD_WIDTH_MIN;    
    bKneadMin = 0;
    bKneadMid = 1; 
    bKneadMax = 1;
  }
  if(M3Dsignal&BIT1)
  {
    by_KneadPosition = KNEAD_WIDTH_MED;    
    bKneadMin = 1;
    bKneadMid = 0; 
    bKneadMax = 1;
  }
  if(M3Dsignal&BIT0)
  {
    by_KneadPosition = KNEAD_WIDTH_MAX;  
    bKneadMin = 1;
    bKneadMid = 1; 
    bKneadMax = 0;
  }
  
  KneadMotor_CalculateSpeed(by_KneadPosition);
  
  if(M3Dsignal&BIT4)
  {
   // bVout = 1;
        bShould = 1;bShouldDelay =1;//nShouldDelay = 0;
  }
  else
  {
   // bVout = 0;
            bShould = 0;
            bShouldDelay =0;
    
  }
  
  if(M3Dsignal&BIT6)
  {
    _3DfrontSwitch = 1;
  }
  else
  {
    _3DfrontSwitch = 0; 
  }
  if(M3Dsignal&BIT5)
  {
    _3DBackSwitch = 1;
  }
  else
  {
    _3DBackSwitch = 0;
  }

  

  SignalBoard_First_signal = SignalBoard_GetFirstMassageSignal();
  //		   7        6	     5              4	           3            2   1   0
  //xxxx xxxx  --> 0 3DbackSwitch 3DfrontSwitch shoulderPosition MotorPosition Max Med Min

   if(SignalBoard_First_signal&BIT4)//ok
   {
     bLegUpSwitch=1;
     
   }
   else
   {
     bLegUpSwitch=0;
     
   }

  if(SignalBoard_First_signal&BIT5)  
   {
     bLegDownSwitch=1;
     
   }
   else
   {
     bLegDownSwitch=0;
     
   }  
  
  
  //-----------------------------------------------------------
 if(SignalBoard_First_signal&BIT6)//靠背限位被反向，原因不明

   {
    
     bBackUpSwitch =1;

   }
   else
   {
     bBackUpSwitch=0;

   }  
   
  if(SignalBoard_First_signal&BIT7)

   {
     bBackDownSwitch=1;

   }
   else
   {
       bBackDownSwitch=0;

   }  
   // two byte signal------------------------------------------------
   
   SignalBoard_Second_signal = SignalBoard_GetSecondSignal();

   if(SignalBoard_Second_signal&BIT0)//ok
   {
     bWalkDownSwitch=1;
     
   }
   else
   {
     bWalkDownSwitch=0;
     
   }  
   if(SignalBoard_Second_signal&BIT1)
   {
     bWalkUpSwitch=1;
     
   }
   else
   {
     bWalkUpSwitch=0;
     
   }  
   
   if(SignalBoard_Second_signal&BIT3)//线束需要调整线序，做相应修改 dyl 20170115
   {
     bFootSwitch=0;//20170509
   }
   else
   {
     bFootSwitch=1; 
   } 
   
   if(SignalBoard_Second_signal&BIT4)
   {
     bGroudSwitch=0;////////////////////////////0170112
   }
   else
   {
     bGroudSwitch=1; 
   }    
   
   if(SignalBoard_Second_signal&BIT5)
   {
     bAngleSwitch=0;//////////////////////////
   }
   else
   {
     bAngleSwitch=1; 
   } 
   
   //--------------------------
 
 if(SignalBoard_Second_signal&BIT7)
   {
     bSlidePadUpSwitch=1;
     
   }
   else
   {
     bSlidePadUpSwitch=0;
     
   }  


  if(SignalBoard_Second_signal&BIT6)
   {
     bSlidePadDownSwitch=1;
     
   }
   else
   {
     bSlidePadDownSwitch=0;
     
   }   
   //three byte signal
   SignalBoard_Three_signal = SignalBoard_GetThreeSignal();
   if(SignalBoard_Three_signal&BIT1)
   {
     bFlexInSwitch=0;//1 20170112
   }
   else
   {
     bFlexInSwitch=1; 
   } 
   if(SignalBoard_Three_signal&BIT0)
   {
     bFlexOutSwitch=0;//1   20170112
   }
   else
   {
     bFlexOutSwitch=1; 
   }   
   
   
   
   
   
   
   /*
     bSlidePadDownSwitch=0;
  bSlidePadUpSwitch=0;
       bWalkUpSwitch=0;
    bWalkDownSwitch=0;
         bBackDownSwitch=0;
       bBackUpSwitch=0;
          bLegDownSwitch=0;
               bLegUpSwitch=0;

          */
          
  //bLegStretchGroundSwitch?Input_High(&st_FlexGround):Input_Low(&st_FlexGround);
  bLegUpSwitch ? Input_High(&st_LegUp):Input_Low(&st_LegUp);
  bLegDownSwitch ? Input_High(&st_LegDown):Input_Low(&st_LegDown);
  
  bBackUpSwitch ? Input_High(&st_BackUp):Input_Low(&st_BackUp);
  
  bBackDownSwitch ? Input_High(&st_BackDown):Input_Low(&st_BackDown);			
  bSlidePadUpSwitch ? Input_High(&st_SlideUp):Input_Low(&st_SlideUp);
  bSlidePadDownSwitch ? Input_High(&st_SlideDown):Input_Low(&st_SlideDown);
  bWalkUpSwitch ? Input_High(&st_WalkUp):Input_Low(&st_WalkUp);
  bWalkDownSwitch ? Input_High(&st_WalkDown):Input_Low(&st_WalkDown);
  
  bFlexInSwitch ? Input_High(&st_FlexIn):Input_Low(&st_FlexIn);
  bFlexOutSwitch ? Input_High(&st_FlexOut):Input_Low(&st_FlexOut);
  
  
  
  
  
  
  
  
//#ifndef RT8600S
  bWalkPulseCount = GPIO_PinInGet(INPUT_WALK_PULSE_PORT, INPUT_WALK_PULSE_BIT);
  if(bPreWalkPulseCount != bWalkPulseCount)
  { 
    if(WalkMotor_GetDirection() != WALK_MOTOR_GO_UP)//马达向下
    {
      if(nCurWalkLocate > 0 )
      {
        nCurWalkLocate-- ;
      }
      if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(0);
      }
    }
    else
    {
      if(nCurWalkLocate < 2000)
      {
        nCurWalkLocate++ ;
      }
      if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
      {
        Input_SetWalkMotorPosition(TOP_POSITION);//1
      }
    }
  }
  bPreWalkPulseCount = bWalkPulseCount;
//#endif
  
  
}
unsigned int Input_GetVout(void)
{

  
    if(bShouldDelay==1) return BODY_TOUCHED; //if(bShould==0) return BODY_TOUCHED;
    return BODY_NO_TOUCHED;
  
}   



//return 0 or 1
unsigned int Input_GetKneadMax(void)
{
  return(bKneadMax);
}
//return 0 or 1
unsigned int Input_GetKneadMid(void)
{
    return(bKneadMid);
}
//return 0 or 1
unsigned int Input_GetKneadMin(void)
{
    return(bKneadMin);
}
unsigned char Input_GetBalance(void)
{
  return(bBalance);
}
unsigned int Input_GetBackUpSwitch(void)
{
  return(st_BackDown.flag );
}                     
unsigned int Input_GetBackDownSwitch(void)
{
  return(st_BackUp.flag);
}                     
unsigned int Input_GetLegUpSwitch(void)
{
  return(st_LegUp.flag);
}                     
unsigned int Input_GetLegDownSwitch(void)
{
  return(st_LegDown.flag);
}

unsigned int Input_GetSlideForwardSwitch(void)
{
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
   //  return(st_SlideDown.flag);
     return(st_SlideUp.flag);
  else
    return(1);
}
unsigned int Input_GetSlideBackwardSwitch(void)
{
  bool enable = ReadEEByte(USER_DATA_BASE + SLIDE_MOTOR_ENABLE_ADDRESS);
  if(enable)
 //  return(st_SlideUp.flag);
    return(st_SlideDown.flag);
  else return(1);
}

unsigned int Input_GetKneadPosition(void)
{
  return (unsigned int)by_KneadPosition;
}

/*
unsigned int Input_GetAxisSW(void)
{
  return(st_AxisSW.flag);
}
*/


unsigned int Input_GetMp3Status(void)
{
  return 1;
}
unsigned int Input_PowerCheck(void)
{
  return 1;
}

/*
bool Input_GetWalkChange(void)
{
  return(bWalkChange);
}

void Input_ClearWalkChange(void)
{
  bWalkChange = 0;
}
*/
unsigned int Input_GetWalkPosition(void)
{
  if(Input_GetWalkDownSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_BOTTOM;
  if(Input_GetWalkUpSwitch() == REACH_WALK_LIMIT)
    return WALK_MOTOR_AT_TOP;
  else
    return WALK_MOTOR_AT_MID;
}

//最前面位置
unsigned int Input_GetFlexOutSwitch(void)
{
  return(st_FlexOut.flag);
} 
//最后面位置
unsigned int Input_GetFlexInSwitch(void)
{
  return(st_FlexIn.flag);
}                     

unsigned int Input_GetFlexFootSwitch(void)
{
  return bFootSwitch;//return(st_Foot.flag);
} 

unsigned int Input_GetFlexAngleSwitch(void)
{
  return bAngleSwitch;//return(st_Angle.flag);
} 
unsigned int Input_GetFlexGroundSwitch(void)
{
 return bGroudSwitch;//return(st_FlexGround.flag);
}


unsigned int Input_GetWalkUpSwitch(void)
{
  return(st_WalkUp.flag);
}
unsigned int Input_GetWalkDownSwitch(void)
{
  return(st_WalkDown.flag);
}
unsigned int Input_GetReady(void)
{
    if(bInputReady) return 1;
    return 0;
}
bool Input_Get3DFrontSwitch(void)
{
  return(_3DfrontSwitch);
} 

bool Input_Get3DBackSwitch(void)
{
  return(_3DBackSwitch);
} 

bool Input_Get_Leg_Press_Status(void)
{
  return(   GPIO_PinInGet(LEG_PRESS_PORT, LEG_PRESS_BIT) );
} 
//获取靠背电动缸的脉冲计数位置
unsigned int Input_GetBackPosition(void)
{
	return nCurBackLocate;
}
void Input_SetBackMotorPosition(unsigned int Position)
{
	nCurBackLocate = Position;
}

void BackMotor_Set_Location(unsigned short locate)
{
   nCurBackLocate = locate;
}







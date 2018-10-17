#ifndef __HOT_FLEX_H__
#define __HOT_FLEX_H__


#define FLEX_MOTOR_ENBL_PORT   gpioPortB
#define FLEX_MOTOR_ENBL_BIT    0
#define FLEX_MOTOR_ENBL_MODE   gpioModePushPull

#define FLEX_MOTOR_PHASE_PORT   gpioPortA
#define FLEX_MOTOR_PHASE_BIT    0
#define FLEX_MOTOR_PHASE_MODE   gpioModePushPull

#define FLEX_MOTOR_DECAY_PORT   gpioPortA
#define FLEX_MOTOR_DECAY_BIT    1
#define FLEX_MOTOR_DECAY_MODE   gpioModePushPull

#define FLEX_MOTOR_FAULT_PORT   gpioPortA
#define FLEX_MOTOR_FAULT_BIT    2
#define FLEX_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define FLEX_MOTOR_RESET_PORT   gpioPortB
#define FLEX_MOTOR_RESET_BIT    3
#define FLEX_MOTOR_RESET_MODE   gpioModePushPull




#define STATE_RUN_FLEX_IN         0  
#define STATE_RUN_FLEX_OUT        1  
#define STATE_FLEX_IDLE           2
#define STATE_RUN_FLEX_RESET      3
#define STATE_RUN_FLEX_MANUAL_OUT 4
#define STATE_RUN_FLEX_TEST_OUT   5
#define STATE_RUN_FLEX_STRETCH_OUT 6

#define FLEX_RUN                0 //正常运行
#define FLEX_STOP_AT_IN         1 //因为碰到最里面行程开关而停止
#define FLEX_STOP_AT_FOOT       2 //因为碰到脚底开关而停止
#define FLEX_STOP_AT_FOOT_LEAVE 3 //因为碰不到脚底开关而停止
#define FLEX_STOP_AT_OUT        4 //因为碰到最外面行程开关而停止
#define FLEX_STOP_AT_IDLE       5 //因为收到停止命令而停止
#define FLEX_STOP_AT_ANGLE      6 //因为角度而停止
#define FLEX_STOP_AT_GROUND     7 //因为角度而停止

#define FLEX_POWER_ON  1
#define FLEX_POWER_OFF 0
/////////////////////

//#define FLEX_MOTOR_TIMER_CUR_CHANNEL 0
//#define FLEX_MOTOR_ROUTE_CUR_EN      TIMER_ROUTE_CC0PEN
#define FLEX_MOTOR_TIMER           TIMER1

#define FLEX_MOTOR_TIMER_CHANNEL     0
#define FLEX_MOTOR_ROUTE_EN          TIMER_ROUTE_CC0PEN
#define FLEX_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC2

#define FLEX_MOTOR_PRESCALE        timerPrescale8
#define FLEX_MOTOR_DEFAULT_TOP     131




#define FLEX_MOTOR_CUR_Timer_CCInit     \
{                                   \
    timerEventEveryEdge,            \
    timerEdgeBoth,                  \
    timerPRSSELCh0,                 \
    timerOutputActionNone,          \
    timerOutputActionNone,          \
    timerOutputActionToggle,        \
    timerCCModePWM,                 \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
} 
#define FLEX_MOTOR_CUR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    FLEX_MOTOR_CUR_PRESCALE,           \
    timerClkSelHFPerClk,            \
    false,                          \
    false,                          \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}
////////////////////////////////////////////////////////////////////////////////


//#define FLEX_MOTOR_CMU_TIMER       cmuClock_TIMER0

#define FLEX_SPEED_STOP     0
#define FLEX_SPEED_SLOW     85
#define FLEX_SPEED_MID      110
#define FLEX_SPEED_FAST     FLEX_MOTOR_DEFAULT_TOP
/*
电流计算方法：（I*0.1*5）/3.3*FLEX_MOTOR_DEFAULT_TOP
*/
#define FLEX_CURRENT_2A    40 
#define FLEX_CURRENT_3A_1  75 
#define FLEX_CURRENT_3A    84//73//62 临时测试
#define FLEX_CURRENT_4A    83
#define FLEX_CURRENT_5A    105
#define FLEX_CURRENT_RESET  FLEX_CURRENT_2A
#define FLEX_CURRENT_DRAG   FLEX_CURRENT_2A
//
#define FLEX_MOTOR_RESET    100 //10 Second 

//140531
#define FLEX_RESET_MAX_TIME   120  //100ms*100 = 10sec
//140602
#define FLEX_TIME_CHANNEL      4

#define FLEX_MOTOR_Timer_CCInit     \
{                                   \
    timerEventEveryEdge,            \
    timerEdgeBoth,                  \
    timerPRSSELCh0,                 \
    timerOutputActionNone,          \
    timerOutputActionNone,          \
    timerOutputActionToggle,        \
    timerCCModePWM,                 \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
} 

#define FLEX_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    FLEX_MOTOR_PRESCALE,            \
    timerClkSelHFPerClk,            \
    timerInputActionNone,           \
    timerInputActionNone,           \
    timerModeUp,                    \
    false,                          \
    false,                          \
    false,                          \
    false,                          \
}



enum
{
  FLEX_MOTOR_CURRENT_HIGH,
  FLEX_MOTOR_CURRENT_LOW
};
enum
{
  FLEX_MOTOR_NORMAL,
  FLEX_MOTOR_FAIL
};


void FlexMotor_Initial_IO(void);
void FlexMotor_Data_Init(void);
void FlexMotor_10ms_Int(void);
void FlexkMotor_Set_Pwm_Data(unsigned long ulDuty);
int FlexPower_On(unsigned char speed);
void FlexPower_Off(void);
unsigned int FlexPower_Get(void);
void FlexMotor_Out(void);
void FlexMotor_In(void);
void FlexMotor_Break(void);
void FlexMotor_Reset(void);
void FlexMotor_Reset_Cancel(void);
int FlexMotor_Get_Fault(void);
unsigned char FlexMotor_Control(unsigned char nFinalFlexPadMotorState,unsigned char speed,unsigned char current);
void FlexMotorFollowingFood(void);
int FlexMotorGetEnable(void);
void FlexMotorSetEnable(void);//
void FlexMotorSetDisable(void);
void Clear_Accident_flag(void);
void Clear_Accident1_flag(void);
unsigned char Get_CurAccident_flag(void);
extern unsigned int Flex_In_Fault;
void FlexMotor_100ms_Int(void);
void nSet_StretchTime( unsigned char time);
unsigned char  nGet_StretchTime( void);
#endif


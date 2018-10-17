#ifndef __HOT_ROOLER_H__
#define __HOT_ROOLER_H__

#define ROLLER_MOTOR_ENBL_PORT   gpioPortB
#define ROLLER_MOTOR_ENBL_BIT    1
#define ROLLER_MOTOR_ENBL_MODE   gpioModePushPull

#define ROLLER_MOTOR_PHASE_PORT   gpioPortB
#define ROLLER_MOTOR_PHASE_BIT    4
#define ROLLER_MOTOR_PHASE_MODE   gpioModePushPull

#define ROLLER_MOTOR_FAULT_PORT   gpioPortB
#define ROLLER_MOTOR_FAULT_BIT    6
#define ROLLER_MOTOR_FAULT_MODE   gpioModeInputPullFilter

#define ROLLER_MOTOR_DECAY_PORT   gpioPortB
#define ROLLER_MOTOR_DECAY_BIT    5
#define ROLLER_MOTOR_DECAY_MODE   gpioModePushPull

#define ROLLER_MOTOR_RESET_PORT   gpioPortC
#define ROLLER_MOTOR_RESET_BIT    0
#define ROLLER_MOTOR_RESET_MODE   gpioModePushPull

#define ROLLER_MOTOR_TIMER           TIMER1
#define ROLLER_MOTOR_TIMER_CHANNEL   1
#define ROLLER_MOTOR_ROUTE_EN        TIMER_ROUTE_CC1PEN
#define ROLLER_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC2

/*
#define ROLLER_MOTOR_TIMER           TIMER3
#define ROLLER_MOTOR_TIMER_CHANNEL   1
#define ROLLER_MOTOR_ROUTE_EN        TIMER_ROUTE_CC1PEN
#define ROLLER_MOTOR_ROUTE_LOCATION  TIMER_ROUTE_LOCATION_LOC0
*/



#define ROLLER_MOTOR_PRESCALE        timerPrescale4
#define ROLLER_MOTOR_DEFAULT_TOP     131

#define ROLLER_MOTOR_Timer_CCInit     \
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

#define ROLLER_MOTOR_Timer_Init       \
{                                   \
    true,                           \
    true,                           \
    ROLLER_MOTOR_PRESCALE,           \
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

enum
{
  ROLLER_MOTOR_CURRENT_HIGH,
  ROLLER_MOTOR_CURRENT_LOW
};
enum
{
  ROLLER_MOTOR_NORMAL,
  ROLLER_MOTOR_FAIL
};

#define ROLLER_MOTOR_PWM_CLOSE      0
#define ROLLER_MOTOR_PWM_OPEN       1

#define ROLLER_MOTOR_PWM_20KHZ      20000

#define ROLLER_MOTOR_POLAR_NEGATIVE  0
#define ROLLER_MOTOR_POLAR_POSITIVE  1
//#define HOT_ROLLER_DEFAULT_TOP  130

#define ROLLER_SET_VOLTAGE    250000 //扩大10000倍

extern unsigned int displayPWM;

////////////////////////////////////////////////////////////////////////////////





#define ROLLER_LEVEL_STOP 0
#define ROLLER_LEVEL_SLOW 1
#define ROLLER_LEVEL_MID  2
#define ROLLER_LEVEL_FAST 3

#define ROLLER_SPEED_STOP     0
#define ROLLER_SPEED_SLOW     75
#define ROLLER_SPEED_MID      95
#define ROLLER_SPEED_FAST     ROLLER_MOTOR_DEFAULT_TOP

//以下滚轮模式只用于气囊自动程序

#define ROLLER_MODE_CON_IN        0
#define ROLLER_MODE_CON_OUT       1
#define ROLLER_MODE_S_INT_IN	  2
#define ROLLER_MODE_S_INT_OUT	  3
#define ROLLER_MODE_L_INT_IN	  4
#define ROLLER_MODE_L_INT_OUT	  5
#define ROLLER_MODE_S_RUB	  6
#define ROLLER_MODE_L_RUB	  7

#define ROLLER_AUTO  1
#define ROLLER_MANUAL 0

#define ROLLER_TO_IN  0
#define ROLLER_TO_OUT 1

#define ROLLER_ON  1
#define ROLLER_OFF 0

//void Roller_SetSpeed(unsigned char rollerSpeed);
//unsigned char Roller_GetSpeed(void);
//void Roller_SetMode(unsigned int mode);
//unsigned char Roller_GetMode(void);
//void RollerMotor_Control(unsigned int speed,unsigned int phase);



void Rooler_Initial_IO(void);
bool RollerMotor_IsRun(void);
void RollerMotor_Set_Current(int current);
void RollerMotor_ClockRun(void);
void RollerMotor_UnClockRun(void);
void RollerMotor_Break(void);
void RollerMotor_Reset(void);
void RollerMotor_Reset_Cancel(void);
int RollerMotor_Get_Fault(void);
void RollerMotor_Control(unsigned int ulDuty,unsigned int phase);
unsigned long RollerMotor_Get_Pwm_Data(void);
void RollerMotor_Set_Pwm_Data(unsigned long ulDuty);

void Roller_SetSpeed(unsigned char rollerSpeed);
unsigned char Roller_GetSpeed(void);
void RollerMotor_Controlstrtch(unsigned int speed,unsigned int phase);
#endif


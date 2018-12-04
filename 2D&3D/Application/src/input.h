
#ifndef __INPUT_H__
#define __INPUT_H__


#define INPUT_KNEAD_MAX_PORT   gpioPortC
#define INPUT_KNEAD_MAX_BIT    13
#define INPUT_KNEAD_MAX_MODE   gpioModeInputPullFilter

#define INPUT_KNEAD_MID_PORT   gpioPortC
#define INPUT_KNEAD_MID_BIT    12
#define INPUT_KNEAD_MID_MODE   gpioModeInputPullFilter

#define INPUT_KNEAD_MIN_PORT   gpioPortC
#define INPUT_KNEAD_MIN_BIT    7
#define INPUT_KNEAD_MIN_MODE   gpioModeInputPullFilter

#define INPUT_SHOULDER_PULSE_PORT   gpioPortC
#define INPUT_SHOULDER_PULSE_BIT    6
#define INPUT_SHOULDER_PULSE_MODE   gpioModeInputPullFilter




#define INPUT_WALK_PULSE_PORT   gpioPortF//E
#define INPUT_WALK_PULSE_BIT    9//8//3//14
#define INPUT_WALK_PULSE_MODE   gpioModeInputPullFilter

#define INPUT_BACK_PULSE_PORT   gpioPortD//F
#define INPUT_BACK_PULSE_BIT    9
#define INPUT_BACK_PULSE_MODE   gpioModeInputPullFilter

#define LEG_PRESS_PORT   gpioPortD
#define LEG_PRESS_BIT    10
#define LEG_PRESS_MODE   gpioModeInputPullFilter
//GPIO_PinInGet(LEG_PRESS_PORT, LEG_PRESS_BIT)
//#define INPUT_AXIS_SW_PORT   gpioPortF
//#define INPUT_AXIS_SW_BIT    4
//#define INPUT_AXIS_SW_MODE   gpioModeInputPullFilter

//#define INPUT_LEG_FOOT_PORT   gpioPortB
//#define INPUT_LEG_FOOT_BIT    3
//#define INPUT_LEG_FOOT_MODE   gpioModeInput

#define KNEAD_WIDTH_UNKNOWN		0
#define KNEAD_WIDTH_MIN			1
#define KNEAD_WIDTH_MED			2
#define KNEAD_WIDTH_MAX			3

//达到极限位置的信号电平
#define REACH_WALK_LIMIT    1   // 0   //hull
#define REACH_BACK_LIMIT    1   //limit switch
#define REACH_LEG_LIMIT     1   //limit switch
#define REACH_SLIDE_LIMIT   1   //limit switch
#define REACH_AXIS_LIMIT    1   //limit switch
#define REACH_FLEX_LIMIT    1   //0   //hull

#define WALK_MOTOR_AT_MID     0
#define WALK_MOTOR_AT_BOTTOM  1
#define WALK_MOTOR_AT_TOP     2

#define LEGSTRETCH_SWITCH_ON	0			
#define LEGSTRETCH_SWITCH_OFF	1

#define LEGGROUND_SWITCH_ON	1 //碰到地面了		
#define LEGGROUND_SWITCH_OFF	0 //未碰到地面，处于悬空状态		

#define LEGANGLE_SWITCH_ON	0 //小腿托盘与垂直线的角度超小于15度时，已经到了危险角度，不能再先前延伸
#define LEGANGLE_SWITCH_OFF	1 //小腿托盘与垂直线的角度超大于15度时，可以向前延伸

#define FOOT_SWITCH_ON		0 //碰到脚了
#define FOOT_SWITCH_OFF		1
void Input_Initial_IO(void);
void Input_5ms_Int(void);
void Input_Proce(void);
unsigned int Input_GetBackUpSwitch(void);
unsigned int Input_GetBackDownSwitch(void);
unsigned int Input_GetLegUpSwitch(void);
unsigned int Input_GetLegDownSwitch(void);
unsigned int Input_GetWalkUpSwitch(void);
unsigned int Input_GetWalkDownSwitch(void);

unsigned int Input_GetKneadPosition(void);
unsigned int Input_GetKneadMax(void);
unsigned int Input_GetKneadMid(void);
unsigned int Input_GetKneadMin(void);
unsigned int Input_GetMp3Status(void);
unsigned int Input_PowerCheck(void);
//bool Input_GetWalkChange(void);
//void Input_ClearWalkChange(void);
void Input_SetWalkMotorPosition(unsigned short locate);
unsigned short Input_GetWalkMotorPosition(void);
//void Input_SetAxisMotorPosition(unsigned short locate);
unsigned short Input_GetAxisMotorPosition(void);
//void Input_SetCounterWalkMotorPosition(unsigned short locate); 
//unsigned short Input_GetCounterWalkMotorPosition(void);
void Input_SetCounterAxisMotorPosition(unsigned short locate); 
unsigned short Input_GetCounterAxisMotorPosition(void);
unsigned int Input_GetWalkPosition(void);
unsigned int Input_GetSlideForwardSwitch(void);
unsigned int Input_GetSlideBackwardSwitch(void);
//unsigned int Input_GetAxisSW(void);
unsigned int Input_GetReady(void);
bool Input_Get3DFrontSwitch(void);
bool Input_Get3DBackSwitch(void);

unsigned char Input_GetBalance(void);
void KneadMotor_CalculateSpeed(unsigned int kneadPosition);


unsigned int Input_GetBackPosition(void);
void Input_SetBackMotorPosition(unsigned int Position);

unsigned int Input_GetVout(void);

void Input_Back_Pulse1MS(void);

unsigned int Input_GetFlexInSwitch(void) ;
unsigned int Input_GetFlexOutSwitch(void);
unsigned int Input_GetFlexGroundSwitch(void);                 
unsigned int Input_GetFlexAngleSwitch(void);
unsigned int Input_GetFlexFootSwitch(void);
bool Input_Get_Leg_Press_Status(void);
void BackMotor_Set_Location(unsigned short locate);
#endif /*__INPUT_H__*/

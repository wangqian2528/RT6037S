#ifndef __SYSTEM_H__
#define __SYSTEM_H__

//extern bool  bTimer2MS;
extern unsigned int n3Dtimecount;
extern unsigned char n3Dtime;

void System_Initial_IO(void);
void System_Delay_us(uint32_t ulData);
void KnockMotor_10msInt(void);
void Problem_10ms_Int(void);
void VoiceUart_100msInt(void);

void System_clr_Counter(void);

#endif

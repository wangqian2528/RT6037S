
#ifndef __MEMORY_H__
#define __MEMORY_H__

#define MEMORY_LENGTH 13
//#define PRODUCT_ID_ADDR         ((uint32_t) (128*1024-5)) //((uint32_t) 0x0000FFFBUL) 
#define USER_DATA_BASE          ((uint32_t) 0x0FE00000UL)  /**< user data flash base address  */
//以下地址是按照字节存放
#define SOFT_MAIN_VER_ADDRESS         0
#define SOFT_SECONDARY_VER_ADDRESS    1
#define SETTLE_ADDRESS                2  //按摩完成是否复位 数据为1复位
#define AIRBAG_STRETCH_ADDRESS        3   //按摩椅内部气囊力度
#define SLIDE_MOTOR_ENABLE_ADDRESS    4   //滑动马达使能与禁止
#define PROGRAM_ENABLE_ADDRESS        5   //编程使能地址
#define DEFAULT_TIME_ADDRESS          6   //程序默认时间地址
#define BLUETOOTH_STATUS_ADDRESS      7   //程序蓝牙状态
#define REST_SLEEP_MODE_ADDRESS       8
#define DEMO_RUN_ON     0x5A

#define ACC_TIME_0_ADDRESS  0x10
#define ACC_TIME_1_ADDRESS  0x11
#define ACC_TIME_2_ADDRESS  0x12
#define ACC_TIME_3_ADDRESS  0x13

#define MEMORY_LENGTH_OF_BYTES      14
#define PROGRAM_FLAG               'p'
#define PROGRAM_BY_BLUETOOTH_FLAG  'l'
#define SOFT_MAIN_VER               1
#define REST_SLEEP_DEFAULT     0// 1 30MIN; 0 20MIN
//2.2 增加复位时按任意键可以停止
//   调整电动伸缩小腿的力度在手动时为3A在复位时为2A

#define SOFT_SECONDARY_VER     1// 20150108改为1.02版
//1.16  靠背 小腿 行走 前滑 调整马达的工作电压到24V 增加加密程序 修复急关机时再开机抖动的问题
/*20170612

+24V  BLUE  BED  GREEN

*/
//-------------------------------------------------------
/*
0: 标识ID
1: 标识ID
2: 气囊强度
3: 拉退强度
4: 关机回位
*************************************/
#define MEMORY_DEFAULT_AIR     1 //0,1,2
#define MEMORY_DEFAULT_SETTLE  0 //0: 运行结束关机不复位,1:运行结束关机复位
#define SLIDE_DEFAULT_ENABLE   1 //拉退状态 5为行程开关
#define BLUETOOTH_STATUS_DEFAULT  1  //1 为开 0为关
unsigned char ReadEEByte(unsigned int nAddress);
void MEM_Write_Memory(PUINT32 pw_Buffer,int numBytes);
void MEM_Read_Memory(PUINT32 pw_Buffer,int numBytes);
#endif
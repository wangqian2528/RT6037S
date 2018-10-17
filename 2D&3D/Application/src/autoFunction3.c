/*
工作减压 为职场人士匠心打造独特按摩程序，根据职场阶层人士长期伏案工作出差等特点、舒缓筋骨、恢复体力。
*/
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoFunction3[] = 
{
  
  
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_1_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_2_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_3_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_4_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_5_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_6_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_7_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_8_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_9_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_WAIST,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
   //揉捶同步到肩部
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  //指压到顶
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,10},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  //轻敲到肩部
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,60,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,60,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  //揉捏
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  //指压到顶
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  
  //行进式揉捏到底
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_1_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_3_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_5_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_7_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_9_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_WAIST,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  //腰部定点敲打
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_WAIST,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,5,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},

} ;


/*
工作减压 为职场人士匠心打造独特按摩程序，根据职场阶层人士长期伏案工作出差等特点、舒缓筋骨、恢复体力。
*/
const WALK_KNEAD_KNOCK_MOTOR_STRUCT_AUTO AutoFunction3_old[] = 
{
  
  
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_1_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_2_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_3_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_4_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_5_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_6_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_7_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_8_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_SHOULDER_WAIST_9_10,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,5},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_WAIST,0,KNEAD_STOP_AT_MED,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,5},
  
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,30},
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_PARK,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,30},
   //揉捶同步到肩部
  {BACK_SUB_MODE_WAVELET,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_RUN,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  //指压到顶
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,10},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  //轻敲到肩部
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,60,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,60,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  //揉捏
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  //指压到顶
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_SHOULDER,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,1,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_PressNeck,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_2,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_1,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  //搓背  
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_PressNeck,0,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_RUBBING,WALK_LOCATE_ABSULATE,WAIST_POSITION,KNEAD_RUN_RUBBING,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  
  
  //行进式揉捏到底
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_NeckMed,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_SHOULDER,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_1_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_3_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_5_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_7_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_SHOULDER_WAIST_9_10,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_WAIST,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_ABSULATE,0,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,20,KNEAD_RUN,0,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  //腰部定点敲打
  {BACK_SUB_MODE_PRESS,WALK_LOCATE_WAIST,0,KNEAD_STOP,0,KNOCK_STOP,0,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MED,5,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_6,_3D_PROGRAM,AXIS_STRONGEST,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,40,0,SPEED_5,_3D_PROGRAM,AXIS_STRONGER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,35,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNOCK,WALK_LOCATE_PARK,0,KNEAD_STOP,0,KNOCK_RUN,30,0,SPEED_3,_3D_PROGRAM,AXIS_WEAKER,_3D_SPEED_5,0},
  {BACK_SUB_MODE_KNEAD,WALK_LOCATE_PARK,0,KNEAD_RUN_STOP_AT_MIN,5,KNOCK_STOP,0,0,SPEED_4,_3D_PROGRAM,AXIS_MIDDLE,_3D_SPEED_5,0},

} ;


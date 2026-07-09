#include "app_task.h"

bool flag_send=false;

void App_Task_Init(void)
{
//定时器初始化
 AttachInterrupt_TIM(&htim7, Task_it_callback);
 HAL_TIM_Base_Start_IT(&htim7);

// IMU 初始化
 DWT_Init(168);
 while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
     ;
 INS_Init();

//抬升机构初始化
 car_lift_init();

 //底盘初始化

 Chassis_Init();
 //升降台
 lift_table_init();


 //三自由度吸盘初始化
 dev_3dof_sc_init();

 //气动手指部分初始化
 air_finger_init();
 Servo_Init();

//通讯初始化
 usb_pc_init();




}

void App_Task_Run(void)
{
 // ---- INS 姿态解算：DWT 计时，≥2ms 调用一次 ----
 static uint32_t ins_tick = 0;
 if (DWT_GetDeltaT64(&ins_tick) >= 0.002f)
 {
     INS_Task();
 }

 static car_motion_t last_motion = CAR_STOP;
 static uint16_t  last_data=0;
if ( usb_pc_run(&current_command))
{
 if (current_command.motion != last_motion)
 {
  // 航向锁定：直行时锁定，旋转/停止时解锁
  switch (current_command.motion)
  {
   case CAR_FORWARD:
   case CAR_BACKWARD:
   case CAR_TRANSLATE_LEFT:
   case CAR_TRANSLATE_RIGHT:
    Chassis_SetHeadingLock(1);
    break;
   case CAR_STOP:
   case CAR_TURN_LEFT:
   case CAR_TURN_RIGHT:
   default:
    Chassis_SetHeadingLock(0);
    break;
  }

  last_motion = current_command.motion;
  flag_send = true;
 }
 else (current_command.data != last_data);
 {
  last_data = current_command.data;
  flag_send = true;
 }

 switch (current_command.motion)
 {
  case CAR_STOP:
   Chassis_SetSpeed(0, 0, 0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car stopped\n", 12);

  }
   break;
  case CAR_FORWARD:
   Chassis_SetSpeed(current_command.data*SPEED_MAX/10000, 0, 0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car forward\n", 12);

  }
   break;
  case CAR_BACKWARD:
   Chassis_SetSpeed(-current_command.data*SPEED_MAX/10000, 0, 0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car backward\n", 13);

  }
   break;
  case CAR_TURN_LEFT:
   Chassis_SetSpeed(0,0,current_command.data*ANGULAR_VELOCITY_MAX/10000);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car turn left\n", 14);

  }
   break;
  case CAR_TURN_RIGHT:
   Chassis_SetSpeed(0,0,-current_command.data*ANGULAR_VELOCITY_MAX/10000);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car turn right\n", 15);

  }
   break;
  case CAR_TRANSLATE_LEFT:
   Chassis_SetSpeed(0,current_command.data*SPEED_MAX/10000, 0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car translate left\n", 19);

  }
   break;
  case CAR_TRANSLATE_RIGHT:
   Chassis_SetSpeed(0, -current_command.data*SPEED_MAX/10000,0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car translate right\n", 20);

  }
   break;
  case CAR_UP:
  if (current_command.data == 55555)
  {
   car_block_up();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"car block up\n", 13);
   }
  }
  else if (current_command.data == 44444)
  {
   car_lift_up();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"car lift up\n", 12);
   }
  }
  break;
  case CAR_DOWN:
  if (current_command.data == 55555)
  {
   car_block_down();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"car block down\n", 15);
   }
  }
  else if (current_command.data == 44444)
  {
   car_lift_down();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"car lift down\n", 14);
   }
  }
  break;
 /*case CAR_DOFF:
  if (current_command.data>140) current_command.data = 140;
  dev_3dof_sc_angle(&sc_motor[0], 140-current_command.data);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"doff set\n", 10);

  }
  break;*/
 case CAR_DOFS:
  dev_3dof_sc_angle(&sc_motor[1], current_command.data);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"dofs set\n", 10);

  }
  break;
 case CAR_DOFT:
  dev_3dof_sc_angle(&sc_motor[2], current_command.data);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"doft set\n", 10);

  }
  break;
 case CAR_SUCK:
  if (current_command.data ==  55555)
  {
   dev_3dof_sc_suck_vacuum();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"suck vacuum\n", 13);
   }
  }
  else if (current_command.data == 44444)
  {
   dev_3dof_sc_suck_blow();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"suck blow\n", 12);
   }
  }
  break;
  case CAR_TABLE_UP:
   lift_table_run(-current_command.data);
  if (flag_send)  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"table up\n", 10);
  }
  break;


  case CAR_TABLE_DOWN:
   lift_table_run(current_command.data);
  if (flag_send)  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"table down\n", 12);
  }
  break;

 case CAR_FINGER:
  if (current_command.data == 55555)
  {
    air_finger_grip();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"finger grip\n", 13);
   }
  }
  else if (current_command.data == 44444)
  {
    air_finger_release();
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"finger release\n", 16);
   }
  }
  break;
 case CAR_FINGER_WRIST:

  if (current_command.data > 220)
  {current_command.data = 220;}
  Servo_SetAngle_135(TIM_CHANNEL_1, 110-current_command.data);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"finger wrist set\n", 17);
  }

  break;


  default:
   break;
 }






 }



}





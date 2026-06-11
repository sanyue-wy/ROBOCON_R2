#include "app_task.h"

bool flag_send=false;

void App_Task_Init(void)
{
//定时器初始化
 AttachInterrupt_TIM(&htim7, Task_it_callback);
 HAL_TIM_Base_Start_IT(&htim7);

//抬升机构初始化
 car_lift_init();

 //底盘初始化

 Chassis_Init();

//通讯初始化
 usb_pc_init();




}

void App_Task_Run(void)
{
 static car_motion_t last_motion = CAR_STOP;
if ( usb_pc_run(&current_command))
{
 if (current_command.motion != last_motion)
 {
  last_motion = current_command.motion;
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
   Chassis_SetSpeed(current_command.data, 0, 0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car forward\n", 12);

  }
   break;
  case CAR_BACKWARD:
   Chassis_SetSpeed(-current_command.data, 0, 0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car backward\n", 13);

  }
   break;
  case CAR_TURN_LEFT:
   Chassis_SetSpeed(0,0,current_command.data);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car turn left\n", 14);

  }
   break;
  case CAR_TURN_RIGHT:
   Chassis_SetSpeed(0,0,-current_command.data);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car turn right\n", 15);

  }
   break;
  case CAR_TRANSLATE_LEFT:
   Chassis_SetSpeed(0,current_command.data, 0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car translate left\n", 19);

  }
   break;
  case CAR_TRANSLATE_RIGHT:
   Chassis_SetSpeed(0, -current_command.data,0);
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car translate right\n", 20);

  }
   break;
  case CAR_UP:
   car_lift_up();
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car up\n", 7);

  }
   break;
  case CAR_DOWN:
   car_lift_down();
  if (flag_send)
  {
   flag_send = false;
   Transmit_to_PC((uint8_t *)"Car down\n", 7);

  }
   break;
  default:
   break;
 }






 }



}








#include "app_task.h"




#if pc_control==1
bool flag_send=false;
#endif

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
#if pc_control==1
 usb_pc_init();
 #endif

#if remote_control==1
 Remote_Init();
 AttachInterrupt_UART_DMA(&huart1,DataBuff,200,Vofa_Callback);
#endif

}

void App_Task_Run(void)
{
 // ---- INS 姿态解算：DWT 计时，≥2ms 调用一次 ----
 static uint32_t ins_tick = 0;
 if (DWT_GetDeltaT64(&ins_tick) >= 0.002f)
 {
  INS_Task();
 }
#if pc_control==1
 static car_motion_t last_motion = CAR_STOP;
 static uint16_t  last_data=0;




 if (  usb_pc_run(&current_command);)

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
  else if (current_command.data != last_data)
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
  case CAR_DOFF:
   if (current_command.data > 180) current_command.data = 180;
   if (current_command.data <= 90)
    dev_3dof_sc_angle(&sc_motor[0], 140 - current_command.data);
   else
    dev_3dof_sc_angle(&sc_motor[0], 140 - (int16_t)(current_command.data - 90));
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"doff set\n", 10);
   }
   break;
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

   if (current_command.data > 180)
   {current_command.data = 180;}
   Servo_SetAngle_135(TIM_CHANNEL_1, 90-current_command.data);
   if (flag_send)
   {
    flag_send = false;
    Transmit_to_PC((uint8_t *)"finger wrist set\n", 17);
   }

   break;


  default:
   break;
  }
#endif

#if remote_control==1

  static uint16_t pre_left_y = 0;

  // SWA总使能检查 - 必须拨上才工作
  if (Remote_control_FS.SWA > 1024)
  {
   // ========== 运动控制（优先级最高） ==========
   // Right_Y → 前后 (速度0~10000, 映射到0~1.592m/s)
   if(Remote_control_FS.Left_X <1000&&Remote_control_FS.Left_Y >1100)
   {
    Chassis_SetHeadingLock(0);
    float vw=(float)((int16_t)Remote_control_FS.Right_Y-1024)/1400.0f*ANGULAR_VELOCITY_MAX;
    Chassis_SetSpeed(0,0,vw);
   }
   else{
    Chassis_SetHeadingLock(1);
    float vx=(float)((int16_t)Remote_control_FS.Right_X-1024)/1400.0f*SPEED_MAX;
    float vy=(float)((int16_t)Remote_control_FS.Right_Y-1024)/1400.0f*SPEED_MAX;
    Chassis_SetSpeed(vx,vy,0);

   }
   // ========== SWC模式切换 ==========
   // SWC低（~240）：Left_Y控制夹爪
   if (Remote_control_FS.SWC < 400)
   {
    // Left_Y最下面（~303）：夹爪松开，舵机默认
    if (Remote_control_FS.Left_Y < 400)
    {
     air_finger_release();

    }
    // Left_Y中间（~800-1200）：夹爪夹紧
    else if (Remote_control_FS.Left_Y > 800 && Remote_control_FS.Left_Y < 1200)
    {
     air_finger_grip();
    }
    // Left_Y最上面（~1765）：旋转+90度
    else if (Remote_control_FS.Left_Y > 1500)
    {
     Servo_SetAngle_135(TIM_CHANNEL_1, -90);
    }
    // Left_Y从最上面回到中间：舵机恢复默认
    else if (Remote_control_FS.Left_Y > 1200 && Remote_control_FS.Left_Y < 1500
             && pre_left_y > 1500)
    {
     air_finger_release();
    }

   }
   // SWC中（~1024）：Left_Y控制升降台，SWB控制车子升降
   else if (Remote_control_FS.SWC > 900 && Remote_control_FS.SWC < 1200)
   {
    // SWB高（~1807）：车子抬升
    if (Remote_control_FS.SWB > 1500)
    {
     car_lift_up();
    }
    // SWB低（~240）：车子下降
    else if (Remote_control_FS.SWB < 500)
    {
     car_lift_down();
    }
    static int16_t lift_target = 0;
    int16_t delta = (int16_t)((float)((int16_t)Remote_control_FS.Left_Y - (int16_t)pre_left_y) / 1400.0f * 300);
    lift_target += delta;
    if (lift_target < 0) lift_target = 0;
    lift_table_run(lift_target);
   }
   // SWC高（~1807）：Left_Y→DOF1, VRA→DOF2, VRB→DOF3, SWD→吸盘
   else if (Remote_control_FS.SWC > 1700)
   {
    // SWD高（~1807）：吸盘吸住
    if (Remote_control_FS.SWD > 1500)
    {
     dev_3dof_sc_suck_blow();
    }
    // SWD低（~240）：吸盘不吸
    else if (Remote_control_FS.SWD < 500)
    {
     dev_3dof_sc_suck_vacuum();
    }
    float DOF[3]={0};
    DOF[0]=(float)((int16_t)Remote_control_FS.Left_Y-1024)/700.0f*45;
    DOF[1]=(float)((int16_t)Remote_control_FS.VRA-1024)/700.0f*2000;
    DOF[2]=(float)((int16_t)Remote_control_FS.VRB-1024)/700.0f*180;

    for ( int i=0;i<3;i++)
    {
     dev_3dof_sc_angle(&sc_motor[i],DOF[i]);

    }
   }


   else
    Chassis_SetSpeed(0,0,0);
   pre_left_y = Remote_control_FS.Left_Y;
  }

  float a[10]={Remote_control_FS.Left_X,Remote_control_FS.Left_Y,Remote_control_FS.Right_X,Remote_control_FS.Right_Y,Remote_control_FS.SWA,Remote_control_FS.SWB,Remote_control_FS.SWC,Remote_control_FS.SWD,Remote_control_FS.VRA,Remote_control_FS.VRB};
  vofa_FloatSend(a,10);

#endif

 }

//
// Created by wy on 2026/4/25.
//

#include "debug.h"

//数据结构区域
#if vofa_debug==1

#endif

#if dji_motor_debug==1

#endif

#if imu_debug==1



#endif

#if lift_debug==1

#endif

#if remote_debug==1

#endif

#if USB_debug==1

#endif
#if pc_debug==1

#endif

#if air_finger_wrist_debug==1
#endif

#if dof_debug==1


#endif

void debug_init(void)
{
    AttachInterrupt_TIM(&htim7, Task_it_callback);
    HAL_TIM_Base_Start_IT(&htim7);
#if USB_debug==1

#endif

#if lift_debug==1

car_lift_init();
    AttachInterrupt_UART_DMA(&huart3,Rx_buf,64,Remote_callback);
#endif


#if remote_debug==1
    AttachInterrupt_UART_DMA(&huart1,DataBuff,200,Vofa_Callback);
    AttachInterrupt_UART_DMA(&huart3,Rx_buf,64,Remote_callback);



#endif


#if imu_debug==1
    DWT_Init(168);
    while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR)
        ;
    INS_Init();

#endif

#if vofa_debug==1

    AttachInterrupt_UART_DMA(&huart1,DataBuff,200,Vofa_Callback);


#endif

#if dji_motor_debug==1

    //底盘初始化
    Chassis_Init();
    //升降台
    lift_table_init();
    //三自由度吸盘初始化
    dev_3dof_sc_init();
    HAL_Delay(4000);

    DJ_SetSpeed(&chassis.ChassisMotors[0], 1000);
    // DJ_SetSpeed(&chassis.ChassisMotors[1], chassis.Motors_Speed[1]);
    // DJ_SetSpeed(&chassis.ChassisMotors[2], chassis.Motors_Speed[2]);
    // DJ_SetSpeed(&chassis.ChassisMotors[3], chassis.Motors_Speed[3]);

    //vofa初始化
    //AttachInterrupt_UART_DMA(&huart1,DataBuff,200,Vofa_Callback);


#endif


#if pc_debug==1
usb_pc_init();
#endif

#if air_finger_wrist_debug==1
    Servo_Init();
   //  HAL_Delay(1000);
   //  Servo_SetAngle_135(TIM_CHANNEL_1,60);
   //  HAL_Delay(1000);
   // Servo_SetAngle_135(TIM_CHANNEL_1,-60);



#endif

#if dof_debug==1
    AttachInterrupt_CAN(&hcan2, DJ_CAN_Callback,15);
    DJ_Init(&sc_motor[2], 3, M3508, PID_METHOD, &hcan2);
    DJ_SetAngleInc(&sc_motor[2], 0);
    HAL_Delay(1000);
    DJ_SetAngleInc(&sc_motor[2], 360);
    HAL_Delay(1000);
    DJ_SetAngleInc(&sc_motor[2], -360);
#endif


}



//死循环内运行的

void debug_run(void)
{
#if air_finger_wrist_debug==1



#endif

#if dof_debug==1


#endif


#if pc_debug==1
    usb_pc_run(&current_command);
    if (current_command.motion == CAR_FORWARD)
    {
       Transmit_to_PC((uint8_t *)"Moving forward\n", 15);

    }


#endif
#if USB_debug==1
CDC_Transmit_FS("hello world\n",11);
    HAL_Delay(1000);
#endif
#if lift_debug==1
if (Remote_control_FS.SWA > 1000)
  car_lift_up();
   if (Remote_control_FS.SWA>0&&Remote_control_FS.SWA<1000)
       car_lift_down();



#endif

#if remote_debug==1

    float a[4]={Remote_control_FS.Left_X,Remote_control_FS.Left_Y,Remote_control_FS.Right_X,Remote_control_FS.Right_Y};
    vofa_FloatSend(a,4);

#endif

#if imu_debug==1
INS_Task();
    //float imu_date[6]={ INS.Accel[X],INS.Accel[Y],INS.Accel[Z],INS.Gyro[X],INS.Gyro[Y],INS.Gyro[Z]};
    //vofa_FloatSend(imu_date,6);
float posture_date[4]={INS.Yaw,INS.Pitch,INS.Roll,INS.YawTotalAngle};
vofa_FloatSend(posture_date,4);

#endif



#if vofa_debug==1

   // float  target=0;
   //   target=ControlLoop();
   //   DJ_SetSpeed(&DJ_Motor3508[0],target);
    uart_pid_to_pid_update(&DJ_Motor3508[0].PID_Speed.Kp,&DJ_Motor3508[0].PID_Speed.Ki,&DJ_Motor3508[0].PID_Speed.Kd,&DJ_Motor3508[0].setSpeed,0);
    float a[5]={DJ_Motor3508[0].PID_Speed.Kp,DJ_Motor3508[0].PID_Speed.Ki,DJ_Motor3508[0].PID_Speed.Ki,DJ_Motor3508[0].setSpeed,DJ_Motor3508[0].speed};
vofa_FloatSend(a,5);

#endif

#if dji_motor_debug==1
     //uart_pid_to_pid_update(&chassis.ChassisMotors[0].PID_Speed.Kp,&chassis.ChassisMotors[0].PID_Speed.Ki,&chassis.ChassisMotors[0].PID_Speed.Kd,&chassis.ChassisMotors[0].setSpeed,0);
    // float a[5]={chassis.ChassisMotors[0].PID_Speed.Kp,chassis.ChassisMotors[0].PID_Speed.Ki,chassis.ChassisMotors[0].PID_Speed.Ki,chassis.ChassisMotors[0].setSpeed,chassis.ChassisMotors[0].speed};
     //vofa_FloatSend(a,5);


#endif






}
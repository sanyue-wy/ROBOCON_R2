#include "Chassis.h"


chassis_t chassis;//底盘参数体

// 底盘初始化
void Chassis_Init(void)
{

    AttachInterrupt_CAN(&hcan1, DJ_CAN_Callback);
#if MOTOR_ON
    DJ_Init(&chassis.ChassisMotors[0], 1, M3508, PID_METHOD);
    DJ_Init(&chassis.ChassisMotors[1], 2, M3508, PID_METHOD);
    DJ_Init(&chassis.ChassisMotors[2], 3, M3508, PID_METHOD);
    DJ_Init(&chassis.ChassisMotors[3], 4, M3508, PID_METHOD);
#endif

    /* 底盘角度pid */
    PID_Init(&chassis.pid, 0.05, 0, 0, 1, 0);
}

inline void Chassis_Run(void)
{
#if MOTOR_ON
    Chassis_SetSpeed(chassis.setVx, chassis.setVy, chassis.setVw);
    DJ_MotorRun();
#endif
}

/**
 * 设置底盘速度
 * 正反向为X轴，左方向为Y轴，m/s
 * 逆时针为正， rad/s
 */
void Chassis_SetSpeed(float Vx, float Vy, float Vw)
{
   float Vp=Vw*(LENGTH + WIDTH);




    chassis.Motors_Speed[0] = (int16_t)((Vx+Vy-Vp) / PI / RADIUS *60 * DECRATIO);
    chassis.Motors_Speed[1] = (int16_t)((Vx-Vy-Vp) / PI / RADIUS *60 * DECRATIO);
    chassis.Motors_Speed[2] = (int16_t)((Vx-Vy+Vp) / PI / RADIUS *60 * DECRATIO);
    chassis.Motors_Speed[3] = (int16_t)((Vx+Vy+Vp) / PI / RADIUS *60 * DECRATIO);

#if MOTOR_ON
    DJ_SetSpeed(&chassis.ChassisMotors[0], chassis.Motors_Speed[0]);
    DJ_SetSpeed(&chassis.ChassisMotors[1], chassis.Motors_Speed[1]);
    DJ_SetSpeed(&chassis.ChassisMotors[2], chassis.Motors_Speed[2]);
    DJ_SetSpeed(&chassis.ChassisMotors[3], chassis.Motors_Speed[3]);
#endif
}


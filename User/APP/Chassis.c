#include "Chassis.h"


chassis_t chassis;//底盘参数体

// 底盘初始化
void Chassis_Init(void)
{
    AttachInterrupt_CAN(&hcan1, DJ_CAN_Callback,0); // 注册CAN中断回调函数,过滤器0
#if MOTOR_ON
    DJ_Init(&chassis.ChassisMotors[0], 1, M3508, PID_METHOD);
    DJ_Init(&chassis.ChassisMotors[1], 2, M3508, PID_METHOD);
    DJ_Init(&chassis.ChassisMotors[2], 3, M3508, PID_METHOD);
    DJ_Init(&chassis.ChassisMotors[3], 4, M3508, PID_METHOD);

    DJ_SetSpeed(&chassis.ChassisMotors[0], chassis.Motors_Speed[0]);
    DJ_SetSpeed(&chassis.ChassisMotors[1], chassis.Motors_Speed[1]);
    DJ_SetSpeed(&chassis.ChassisMotors[2], chassis.Motors_Speed[2]);
    DJ_SetSpeed(&chassis.ChassisMotors[3], chassis.Motors_Speed[3]);


#endif


    PID_Init(&chassis.pid, 2.0f, 0.0f, 0.3f, 3.0f, 0.5f);
}

 void Chassis_Run(void)
{
#if MOTOR_ON
    Chassis_Speed(chassis.setVx, chassis.setVy, chassis.setVw + chassis.pid.out);
#endif
}

/**
 * 航向PID计算 — 仅读 INS.YawTotalAngle + 一次 PID_Calc，极轻量(~200 cycles)
 *
 */
void Chassis_YawControl(void)
{
    if (chassis.heading_lock)
    {
        float current = INS.YawTotalAngle * DEG2RAD;
        float target  = chassis.targetYaw * DEG2RAD;
        PID_Calc(&chassis.pid, current, target);
    }
    else
    {
        chassis.pid.out = 0.0f;
    }
}

/**
 * 锁定当前航向
 * enable=1: 记录当前 YawTotalAngle 为目标，开始闭环
 * enable=0: 解除锁定，回到开环
 */
void Chassis_SetHeadingLock(uint8_t enable)
{
    if (enable)
        chassis.targetYaw = INS.YawTotalAngle;
    chassis.heading_lock = enable;
    chassis.pid.out = 0.0f;
}

/**
 * 旋转到指定角度（绝对角度，单位：度）
 */
void Chassis_SetTargetAngle(float angle_deg)
{
    chassis.targetYaw = angle_deg;
    chassis.heading_lock = 1;
}

void Chassis_SetSpeed(float Vx, float Vy, float Vw)
{
    chassis.setVx = Vx;
    chassis.setVy = Vy;
    chassis.setVw = Vw;
}

/**
 * 设置底盘速度
 * 正方向为X轴，左方向为Y轴，m/s
 * 逆时针为正， rad/s
 */
void Chassis_Speed(float Vx, float Vy, float Vw)
{
   float Vp=Vw*(LENGTH + WIDTH);




    chassis.Motors_Speed[0] = (int16_t)((-Vx-Vy*(1+G_COMPENSATION)-Vp) / PI / RADIUS *60 * DECRATIO);
    chassis.Motors_Speed[1] = (int16_t)((Vx-Vy*(1+G_COMPENSATION)-Vp) / PI / RADIUS *60 * DECRATIO);
    chassis.Motors_Speed[2] = (int16_t)((-Vx+Vy*(1-G_COMPENSATION)-Vp) / PI / RADIUS *60 * DECRATIO);
    chassis.Motors_Speed[3] = (int16_t)((Vx+Vy*(1-G_COMPENSATION)-Vp) / PI / RADIUS *60 * DECRATIO);

#if MOTOR_ON
    DJ_SetSpeed(&chassis.ChassisMotors[0], chassis.Motors_Speed[0]);
    DJ_SetSpeed(&chassis.ChassisMotors[1], chassis.Motors_Speed[1]);
    DJ_SetSpeed(&chassis.ChassisMotors[2], chassis.Motors_Speed[2]);
    DJ_SetSpeed(&chassis.ChassisMotors[3], chassis.Motors_Speed[3]);
#endif
}


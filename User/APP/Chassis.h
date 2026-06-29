/**
* 四轮麦轮底盘
 *
 * 以码盘正方向为x轴正方向，左侧为Y轴正方向
 * -----右前ID为1----- *
 * -----左前ID为2----- *
 * -----左后ID为4----- *
 * -----右后ID为3----- *
 */


#ifndef CHASSIS_H
#define CHASSIS_H
#include "dvc_dji_motor.h"
#include "FastMathFunctions.h"
#include "ins_task.h"

#define REMOTE_ON 1
#define MOTOR_ON 1
#define SELFINSPEDTION_ON 1

#define RADIUS 0.152f // 轮子直径
#define LENGTH 0.4518f // 底盘半长
#define WIDTH 0.45f  // 底盘半宽
#define G_COMPENSATION 0.04f // 实际重心与理论重心偏移造成前后轮平移误差补偿
#define RR 0.6377f   // 轮子到中心的距离
#define DECRATIO 19   // 电机减速比



typedef enum
{
    VELOCITY_MODE,
    POSITION_MODE,
} chassis_ctrl_e;

typedef  struct
{
    DJ_Motor_t ChassisMotors[4];

    int16_t Motors_Speed[4];

    float roll;  // 横滚角， 绕x轴
    float pitch; // 俯仰角，绕y轴
    float yaw;   // 偏航角， 绕z轴

    PID_t pid; // 角度pid
    float setVx;
    float setVy;
    float setVw;

    chassis_ctrl_e ctrlMode;

    float targetYaw;       // 目标航向角 (度，来自 INS.YawTotalAngle)
    uint8_t heading_lock;  // 航向锁定: 1=闭环保持, 0=开环

} chassis_t;

extern  chassis_t chassis;

void Chassis_Init(void);
void Chassis_Run(void);
void Chassis_SetSpeed(float Vx, float Vy, float Vw);

// 航向闭环接口
void Chassis_SetHeadingLock(uint8_t enable);     // 锁定/解锁当前航向
void Chassis_SetTargetAngle(float angle_deg);    // 旋转到指定绝对角度
void Chassis_YawControl(void);                   // 航向PID计算（定时器中调用）







#endif //CHASSIS_H
/**
* 四轮麦轮底盘
 *
 * 以码盘正方向为x轴正方向，左侧为Y轴正方向
 * -----右前ID为2----- *
 * -----左前ID为1----- *
 * -----左后ID为4----- *
 * -----右后ID为3----- *
 */


#ifndef CHASSIS_H
#define CHASSIS_H
#include "dvc_dji_motor.h"
#include "FastMathFunctions.h"

#define REMOTE_ON 1
#define MOTOR_ON 1
#define SELFINSPEDTION_ON 1

#define RADIUS 0.1265f // 轮子直径
#define LENGTH 0.190f // 底盘半长
#define WIDTH 0.1828f  // 底盘半宽
#define RR 0.26366f   // 轮子到中心的距离
#define DECRATIO 19   // 电机减速比

#define MAX_VELOCITY1 1.0f
#define MAX_ANGULAR1 1.0f
#define MAX_VELOCITY2 2.0f
#define MAX_ANGULAR2 1.5f
#define MAX_VELOCITY3 3.5f
#define MAX_ANGULAR3 3.0f


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

} chassis_t;

extern  chassis_t chassis;

void Chassis_Init(void);
void Chassis_Run(void);
void Chassis_SetSpeed(float Vx, float Vy, float Vw);







#endif //CHASSIS_H
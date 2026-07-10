/**
 * @file    wheeled_odometer.h
 * @brief   基于卡尔曼滤波的轮式里程计（四轮麦轮底盘）
 */

#ifndef __WHEELED_ODOMETER_H
#define __WHEELED_ODOMETER_H

#include "stdint.h"
#include "kalman_filter.h"

/*--------------------------------------------------------------
 * 底盘参数（来自Chassis.h）
 *--------------------------------------------------------------*/
#define WHEEL_RADIUS    0.076f   // 轮子半径 (m) = 0.152/2
#define CHASSIS_LENGTH  0.4518f  // 底盘半长 (m)
#define CHASSIS_WIDTH   0.45f    // 底盘半宽 (m)
#define MOTOR_DEC_RATIO 19       // 减速比
#define WHEEL_PERIMETER (2.0f * 3.14159265f * WHEEL_RADIUS)

/* 电机速度转换系数: rpm → m/s */
#define RPM_TO_MS       (WHEEL_PERIMETER / (60.0f * MOTOR_DEC_RATIO))

/*--------------------------------------------------------------
 * 卡尔曼滤波器参数（可根据实际调试修改）
 *--------------------------------------------------------------*/
#define KF_DT           0.002f   // 更新周期 2ms (500Hz)
#define ACCEL_NOISE     1.5f     // IMU加速度过程噪声 (m/s²)
#define ENCODER_NOISE   0.2f     // 编码器测量噪声 (m/s)

/*--------------------------------------------------------------
 * 里程计输出结构体
 *--------------------------------------------------------------*/
typedef struct
{
    /* 位置 (导航坐标系, m) */
    float pos_x;
    float pos_y;

    /* 速度 (导航坐标系, m/s) */
    float vel_x;
    float vel_y;

    /* 角速度 (rad/s) */
    float vel_w;

    /* 航向角 (度) */
    float yaw;
    float yaw_total;

    /* 机体坐标系速度 (m/s) */
    float vel_body_x;
    float vel_body_y;

    /* 状态标志 */
    uint8_t initialized;
} Odometer_t;

/*--------------------------------------------------------------
 * 全局变量
 *--------------------------------------------------------------*/
extern Odometer_t Odometer;

/*--------------------------------------------------------------
 * 函数声明
 *--------------------------------------------------------------*/

/**
 * @brief  初始化里程计（在main中调用一次）
 */
void Odometer_Init(void);

/**
 * @brief  里程计更新（在500Hz定时器中调用）
 */
void Odometer_Update(void);

/**
 * @brief  重置里程计位置和速度
 */
void Odometer_Reset(void);

/**
 * @brief  逆运动学：从电机速度解算底盘速度
 * @param  motor_speed[4]  电机速度数组 (rpm)
 * @param  vx              输出：前进速度 (m/s)
 * @param  vy              输出：侧向速度 (m/s)
 * @param  vw              输出：旋转角速度 (rad/s)
 */
void Chassis_Inverse_Kinematics(float motor_speed[4],
                                 float *vx, float *vy, float *vw);

#endif /* __WHEELED_ODOMETER_H */

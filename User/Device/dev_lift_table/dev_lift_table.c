//
// Created by wy on 2026/6/11.
//

#include "dev_lift_table.h"



DJ_Motor_t lift_motor;
/****
 * 升降台的电机初始化
 * 要求要么放在有CAN回调初始化的函数如底盘初始化后面，要么自己前面搭建一个初始化函数
 *
 */
void lift_table_init(void)
{
    DJ_Init(&lift_motor, 5, M3508, PID_METHOD);
   DJ_SetAngle(&lift_motor, 0,8000);
}

/***
 *
 * 单位毫米mm
 */
void lift_table_run(int16_t high)
{
    int32_t angle_add=(int32_t)((float)high/HIGH_MAP_ANGLE*360);
    DJ_SetAngle(&lift_motor, angle_add,8000);
}



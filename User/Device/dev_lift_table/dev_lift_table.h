//
// Created by wy on 2026/6/11.
//

#ifndef dev_lift_table_H
#define dev_lift_table_H
#include "dvc_dji_motor.h"

#define  HIGH_MAP_ANGLE 14  //电机转一圈上升多少毫米

 extern DJ_Motor_t lift_motor;

void lift_table_init(void);
void lift_table_run(int16_t high);
#endif //dev_lift_table_H
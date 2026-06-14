//
// Created by wy on 2026/6/11.
//

#ifndef dev_3DOF_SC_H
#define dev_3DOF_SC_H
#include "dvc_dji_motor.h"
#include "gpio.h"

extern DJ_Motor_t sc_motor[3];

void dev_3dof_sc_angle(DJ_Motor_t *motor, float angle);
void dev_3dof_sc_init(void);
void dev_3dof_sc_suck_vacuum(void);
void dev_3dof_sc_suck_blow(void);

#endif //dev_3DOF_SC_H
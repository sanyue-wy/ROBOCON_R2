//
// Created by wy on 2026/6/11.
//

#include "dev_3DOF_SC.h"


DJ_Motor_t sc_motor[3];

void dev_3dof_sc_init(void)
{
    DJ_Init(&sc_motor[2], 8, M3508, PID_METHOD);
   DJ_SetAngle(&sc_motor[2], 0,1000);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);

    }

void dev_3dof_sc_angle(DJ_Motor_t *motor, float angle)
{
    DJ_SetAngle(motor, angle,4000);
}

void dev_3dof_sc_suck_vacuum(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}
void dev_3dof_sc_suck_blow(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}
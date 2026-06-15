//
// Created by wy on 2026/6/11.
//

#include "dev_3DOF_SC.h"


DJ_Motor_t sc_motor[3];

void dev_3dof_sc_init(void)
{
    AttachInterrupt_CAN(&hcan2, DJ_CAN_Callback,15);

    DJ_Init(&sc_motor[0], 1, M3508, PID_METHOD, &hcan2);
    DJ_Init(&sc_motor[1], 2, M3508, PID_METHOD, &hcan2);
    DJ_Init(&sc_motor[2], 3, M3508, PID_METHOD, &hcan2);

    DJ_SetAngle(&sc_motor[0], 0,4000);
   DJ_SetAngle(&sc_motor[1], 0,4000);
   DJ_SetAngle(&sc_motor[2], 0,4000);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);

    }

void dev_3dof_sc_angle(DJ_Motor_t *motor, float angle)
{
    DJ_SetAngle(motor, angle,4000);
}

void dev_3dof_sc_suck_vacuum(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}
void dev_3dof_sc_suck_blow(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}
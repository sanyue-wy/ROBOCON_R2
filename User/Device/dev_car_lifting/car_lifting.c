//
// Created by wy on 2026/4/29.
//

#include "car_lifting.h"
void car_lift(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_SET);

}
void car_down(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET);

}
void car_lift_init(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET);

}
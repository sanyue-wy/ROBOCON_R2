//
// Created by wy on 2026/4/29.
//

#include "dev_lift.h"
void car_head_lift_up(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_SET);


}
void car_end_lift_up(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_SET);
}
void car_head_lift_down(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_RESET);
}
void car_end_lift_down(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET);
}

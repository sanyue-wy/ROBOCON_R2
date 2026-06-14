//
// Created by wy on 2026/6/11.
//

#include "dev_air_finger.h"
void air_finger_init(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);


}

void air_finger_grip(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
}
void air_finger_release(void)
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
}

void Servo_Init(void)
{
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);


    /******舵机恢复中位*********/

    Servo_SetAngle_135(TIM_CHANNEL_1,0);
}

/**
 * @brief  适配 TIM8 (PSC=167, ARR=19999) 的 ±135° 舵机控制
 * @param  channel: TIM_CHANNEL_1/2/3
 * @param  angle: -135 ~ +135 度
 */
void Servo_SetAngle_135(uint32_t channel, int16_t angle)
{
    // 限制角度范围 -90 ~ +90
    if (angle > 135)  angle = 135;
    if (angle < -135) angle = -135;

    // 线性映射：-90°(500) ~ +90°(2500)
    // 0° 对应 1500
    uint32_t ccr = 1500 + (angle * 1000) / 135;

    __HAL_TIM_SET_COMPARE(&htim8, channel, ccr);
}


/**
 * @brief  直接设置高电平时间（单位：微秒 us）
 * @param  us: 500 ~ 2500 us
 */
void Servo_SetPulse(uint32_t channel, uint16_t us)
{
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;
    __HAL_TIM_SET_COMPARE(&htim8, channel, us);
}
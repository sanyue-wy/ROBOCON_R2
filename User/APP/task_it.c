//
// Created by wy on 2026/4/26.
//

#include "task_it.h"
/* 毫秒定时器 */
void MM_TIM_Callback(void)
{
    static uint16_t count;
    /***********0.1秒计时器***********/
    if (count++ > 100)
    {
        /***********程序运行指示灯************/
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);

        count = 0;
    }
   // SineGen_Update();
}
void TASK_1MS_TIM_callback(void)
{



    DJ_MotorRun();



}

void Task_it_callback(void)
{
    static uint16_t count;
    DJ_MotorRun();

    if (count++ > 1000)
    {
        count = 0;
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
    }



}

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
    SineGen_Update();
}
void TASK_1MS_TIM_callback(void)
{



    DJ_MotorRun();



}
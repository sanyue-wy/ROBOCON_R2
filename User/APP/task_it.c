//
// Created by wy on 2026/4/26.
//

#include "task_it.h"
#include "Chassis.h"
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
    /* DJ_MotorRun 已移至 Task_it_callback (3ms周期) */
}

void Task_it_callback(void)
{
    static uint16_t count1=0;
    static uint16_t count2=0;
    if (++count2>=2)
    {
        count2 = 0;
        Chassis_YawControl();  // 航向PID（~200 cycles）
        Chassis_Run();         // 应用航向PID修正 + 发送电机指令
    }


    if (++count1 >=1000)
    {
        count1 = 0;
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
    }



}
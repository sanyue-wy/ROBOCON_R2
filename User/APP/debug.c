//
// Created by wy on 2026/4/25.
//

#include "debug.h"


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
}

void debug_init(void)
{
    AttachInterrupt_TIM(&htim7, MM_TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim7);

#ifdef vafa_debug
    AttachInterrupt_UART_DMA(&huart3,Rx_buf,64,Remote_callback);


#endif



}

void debug_run(void)
{

#ifdef vafa_debug


#endif


}
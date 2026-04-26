#include "drv_tim.h"
#include "stdlib.h"
//-------------------------------------------------------------------------------------------------------------------
// TIM Interrupt
#ifdef HAL_TIM_MODULE_ENABLED

static uint8_t TIM_Function_Count = 0;      // 函数数量
static TIM_Interrupt_t *TIM_ItSource_Array; // TIM中断回调函数结构体数组指针

/**
 * 联接TIM中断源和中断回调函数
 * 应输入TIM源, 回调函数个数以及回调函数的地址
 * 联接后会自动开启中断
 */
void AttachInterrupt_TIM(TIM_HandleTypeDef *htim, void (*TIM_Callback)(void))
{
    TIM_ItSource_Array = (TIM_Interrupt_t *)realloc(TIM_ItSource_Array, (TIM_Function_Count + 1) * sizeof(TIM_Interrupt_t));
    TIM_ItSource_Array[TIM_Function_Count].htim = htim;
    TIM_ItSource_Array[TIM_Function_Count].TIM_Callback = TIM_Callback;
    TIM_Function_Count++;
}

/**
 * TIM中断回调
 * 如果更改了系统时钟源，需要在main.c文件中调用此函数
 */
inline void BSP_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < TIM_Function_Count; i++)
    {
        if (TIM_ItSource_Array[i].htim == htim)
        {
            TIM_ItSource_Array[i].TIM_Callback();
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    BSP_TIM_PeriodElapsedCallback(htim);
}

#endif /* HAL_TIM_MODULE_ENABLED */

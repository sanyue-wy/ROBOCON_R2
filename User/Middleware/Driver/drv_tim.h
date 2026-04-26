#ifndef DRV_TIM_H
#define DRV_TIM_H

#include "main.h"
//-----------------------------------------------------------------------------------------------------------------------
// 如果启用TIM base功能
#ifdef HAL_TIM_MODULE_ENABLED
#include "tim.h"

    typedef struct
    {
        TIM_HandleTypeDef *htim;
        void (*TIM_Callback)(void);
    } TIM_Interrupt_t;

extern void AttachInterrupt_TIM(TIM_HandleTypeDef *htim, void (*TIM_Callback)(void));
extern void BSP_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* HAL_TIM_MODULE_ENABLED */

#endif //DRV_TIM_H
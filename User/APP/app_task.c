#include "app_task.h"


void App_Task_Init(void)
{
//定时器初始化
 AttachInterrupt_TIM(&htim7, MM_TIM_Callback);
 HAL_TIM_Base_Start_IT(&htim7);





Chassis_Init();

	init_finished = true;
}

void App_Task_Run(void)
{

}

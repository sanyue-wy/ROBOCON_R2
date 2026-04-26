//
// Created by wy on 2026/4/25.
//

#include "debug.h"

//数据结构区域
#if vofa_debug==1

#endif

#if dji_motor_debug==1

#endif


void debug_init(void)
{
    AttachInterrupt_TIM(&htim7, MM_TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim7);

#if vofa_debug==1

    AttachInterrupt_UART_DMA(&huart1,DataBuff,200,Vofa_Callback);


#endif

#if dji_motor_debug==1
    AttachInterrupt_CAN(&hcan1, DJ_CAN_Callback);
    DJ_Init(&DJ_Motor3508[0], 1, M3508, PID_METHOD);
    DJ_SetSpeed(&DJ_Motor3508[0],0);
    AttachInterrupt_TIM(&htim1, TASK_1MS_TIM_callback);
    HAL_TIM_Base_Start_IT(&htim1);
#endif


}

void debug_run(void)
{

#if vofa_debug==1
   float  target=0;
    target=ControlLoop();
    DJ_SetSpeed(&DJ_Motor3508[0],target);
    float a[5]={DJ_Motor3508[0].PID_Speed.Kp,DJ_Motor3508[0].PID_Speed.Ki,DJ_Motor3508[0].PID_Speed.Ki,DJ_Motor3508[0].setSpeed,DJ_Motor3508[0].speed};
vofa_FloatSend(a,5);

#endif

#if dji_motor_debug==1

#endif

}
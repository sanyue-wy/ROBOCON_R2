//
// Created by wy on 2026/4/25.
//

#ifndef debug_H
#define debug_H
#include "dev_vofa.h"
#include  "drv_tim.h"
#include  "drv_can.h"
#include  "drv_usart.h"
#include  "FastMathFunctions.h"
#include  "task_it.h"
#include  "dvc_dji_motor.h"
#include "ins_task.h"
#include "spi.h"
#include  "bsp_dwt.h"
#include "BMI088driver.h"

//测试模式
#define vofa_debug 0
#define dji_motor_debug 0
#define imu_debug 1

void debug_init(void);
void debug_run(void);



#endif //debug_H
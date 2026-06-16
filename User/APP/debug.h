//
// Created by wy on 2026/4/25.
//
// @deprecated 此文件已弃用，请使用 app_task.h 替代
// 保留此文件仅为向后兼容，新代码请使用 app_task.h 中的配置宏
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
#include  "dev_lift.h"
#include  "dvc_remote.h"
#include  "usbd_cdc_if.h"
#include "dev_pc.h"
#include "dev_air_finger/dev_air_finger.h"
#include "dev_3DOF_SC/dev_3DOF_SC.h"
#include "chassis.h"
#include "dev_lift_table/dev_lift_table.h"


// @deprecated 测试模式 - 请使用 app_task.h 中的 APP_ENABLE_xxx 宏
#define vofa_debug 0
#define dji_motor_debug 1
#define imu_debug 0
#define lift_debug 0
#define remote_debug 0
#define  USB_debug 0
#define  pc_debug 0
#define  air_finger_wrist_debug 0
#define  dof_debug 0

// @deprecated 请使用 App_Task_Init() 和 App_Task_Run()
void debug_init(void);
void debug_run(void);

#endif //debug_H
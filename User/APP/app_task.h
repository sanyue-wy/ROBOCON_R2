#include "dev_pc.h"
#include "car_lifting.h"
#include "Chassis.h"
#include "ins_task.h"
#include "task_it.h"
#include "dev_3DOF_SC.h"
#include "dev_lift_table/dev_lift_table.h"
#include "dev_air_finger/dev_air_finger.h"
#include "bsp_dwt.h"
#include "spi.h"
#include "dvc_remote.h"
#include "dev_vofa.h"

#define remote_control 1
#define pc_control 0
extern pc_command_t current_cmd;
//
#define SPEED_MAX 1.592f // 单位m/s
#define ANGULAR_VELOCITY_MAX 2.49f //  单位 rad/s

void App_Task_Init(void);
void App_Task_Run(void);



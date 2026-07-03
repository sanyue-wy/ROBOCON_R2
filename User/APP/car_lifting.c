//
// Created by wy on 2026/6/11.
//

#include "car_lifting.h"

#include "dev_lift.h"
#include "dev_lift.h"

void car_lift_init(void)
{
    car_end_lift_down();
    car_head_lift_down();
    car_air_control_down();
}
void car_lift_up(void)
{
    car_air_control_up();
    // //前后抬升
    car_end_lift_up();
    car_head_lift_up();
    // HAL_Delay(1000);
    //
    // //走一点点让前面顶住
    // Chassis_SetSpeed(0.2, 0, 0);
    // HAL_Delay(300);
    // Chassis_SetSpeed(0, 0, 0);
    //
    // //前面在上层台阶走一点
    // car_head_lift_down();
    // HAL_Delay(1000);
    // Chassis_SetSpeed(0.4, 0, 0);
    // HAL_Delay(300);
    // Chassis_SetSpeed(0, 0, 0);
    //
    // //最后稳定
    // car_end_lift_down();
    // HAL_Delay(1000);
    // Chassis_SetSpeed(0.4, 0, 0);
    // HAL_Delay(300);
    // Chassis_SetSpeed(0, 0, 0);




}


void car_lift_down(void)
{
   //  //走一点前轮悬空
   //  Chassis_SetSpeed(0.2, 0, 0);
   //  HAL_Delay(300);
   //  Chassis_SetSpeed(0, 0, 0);
   //
   //  //前轮落地，走一点让后轮出来
   //  car_end_lift_up();
   //  HAL_Delay(1000);
   //  Chassis_SetSpeed(0.4, 0, 0);
   //  HAL_Delay(300);
   //  Chassis_SetSpeed(0, 0, 0);
   //
   //  //后轮落地然后走一点出来
   //  car_end_lift_up();
   //  HAL_Delay(1000);
   //  Chassis_SetSpeed(0.4, 0, 0);
   //  HAL_Delay(300);
   //  Chassis_SetSpeed(0, 0, 0);
   //
   car_end_lift_down();
   car_head_lift_down();
   //  HAL_Delay(1000);

}

void car_block_up(void)
{
    car_head_lift_up();
    car_end_lift_up();

}
void car_block_down(void)
{
    car_head_lift_up();
    car_end_lift_up();

}




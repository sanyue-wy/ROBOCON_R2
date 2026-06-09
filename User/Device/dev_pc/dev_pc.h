//
// Created by wy on 2026/5/18.
//

#ifndef dev_pc_H
#define dev_pc_H
#include "drv_usb.h"

//运动状态枚举
typedef enum {
    car_stop = 0,
    car_forward,
    car_backward,
    car_turn_left,
    car_turn_right,
    car_translate_left,
    car_translate_right,
    car_up,
    car_down
} movement_state;

//PC命令解析结果
typedef struct {
    movement_state state;   // 运动状态
    float data[20];         // 命令数据
    uint8_t data_count;     // 有效数据个数
} pc_command_t;

extern pc_command_t current_cmd; // 全局变量，存储最新解析的命令

void usb_pc(void);
void usb_receive_callback(uint8_t *Buffer, uint16_t Length);
void Transmit_to_PC(uint8_t *Data, uint16_t Length);
uint8_t usb_pc_run(pc_command_t *cmd_out);  // 返回1=有新命令, 0=无新数据

#endif //dev_pc_H
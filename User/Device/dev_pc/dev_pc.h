
//
// Created by wy on 2026/5/18.
// 协议格式：帧头(1字节) + 运动状态(4字节) + 数据(5字节ASCII数字，左补零)
//

#ifndef DEV_PC_H
#define DEV_PC_H

#include <stdint.h>
#include <stdbool.h>

#define PC_FRAME_HEADER 'M'

typedef enum {
    CAR_STOP = 0,
    CAR_FORWARD,
    CAR_BACKWARD,
    CAR_TURN_LEFT,
    CAR_TURN_RIGHT,
    CAR_TRANSLATE_LEFT,
    CAR_TRANSLATE_RIGHT,
    CAR_UP,
    CAR_DOWN,
    CAR_AIR_BREAK, //这三个自由度
    CAR_DOFT,
    CAR_SUCK,
    CAR_TABLE_UP,
    CAR_TABLE_DOWN,
    CAR_FINGER,
    CAR_FINGER_WRIST
} car_motion_t;

typedef struct {
    car_motion_t motion;
    uint16_t     data;
    bool         valid;
} pc_command_t;

extern  pc_command_t current_command;
void usb_pc_init(void);                      // 初始化USB（无需回调）
bool usb_pc_run(pc_command_t *out_cmd);      // 尝试获取一个命令，成功返回true
void Transmit_to_PC(uint8_t *Data, uint16_t Length);

#endif
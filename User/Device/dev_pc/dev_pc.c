//
// Created by wy on 2026/5/18.
//

#include "dev_pc.h"

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#include "usart.h"

#define APP_RX_BUF_NUM  4
static uint8_t app_rx_buf[APP_RX_BUF_NUM][USB_BUFFER_SIZE];
static uint16_t app_rx_len[APP_RX_BUF_NUM];
static uint8_t app_rx_wp = 0; // 写指针（回调写入）
static uint8_t app_rx_rp = 0; // 读指针（主循环读取）



//USB接受初始化
void usb_pc(void)
{
    USB_Init(usb_receive_callback);
}

void Transmit_to_PC(uint8_t *Data, uint16_t Length)
{
    if (Data!=NULL)
    USB_Transmit_Data( Data,  Length);
}
//usb接受回调函数
void usb_receive_callback(uint8_t *Buffer, uint16_t Length)
{
    // 仅做：数据拷贝 + 入队（1~2ms完成，绝对不阻塞USB接收）
    if (Length == 0 || Length > USB_BUFFER_SIZE) return;

    // 写入环形缓冲区
    memcpy(app_rx_buf[app_rx_wp], Buffer, Length);
    app_rx_len[app_rx_wp] = Length;

    // 移动写指针
    app_rx_wp = (app_rx_wp + 1) % APP_RX_BUF_NUM;
}

//字符串 → 运动状态枚举
static movement_state Parse_State(const char *str)
{
    if (strcmp(str, "car_forward") == 0)        return car_forward;
    if (strcmp(str, "car_backward") == 0)       return car_backward;
    if (strcmp(str, "car_turn_left") == 0)      return car_turn_left;
    if (strcmp(str, "car_turn_right") == 0)     return car_turn_right;
    if (strcmp(str, "car_translate_left") == 0) return car_translate_left;
    if (strcmp(str, "car_translate_right") == 0)return car_translate_right;
    if (strcmp(str, "car__up") == 0)        return car_up;
    if (strcmp(str, "car__down") == 0)      return car_down;
    return car_stop;
}

//实际的命令解析  格式: "运动状态:数据1,数据2,..."
//解析结果通过cmd_out传出，不依赖全局变量
static void USB_Parse_Data(pc_command_t *cmd, uint8_t *Data, uint16_t Len)
{
    char cmd_state[10] = {0};
    char cmd_info[20] = {0};
    uint8_t colon_pos = 0;
    uint8_t has_colon = 0;
    uint8_t i;

    // 初始化输出
    cmd->state = car_stop;
    memset(cmd->data, 0, sizeof(cmd->data));
    cmd->data_count = 0;

    // 1. 查找冒号位置
    for (i = 0; i < Len; i++)
    {
        if (Data[i] == ':')
        {
            colon_pos = i;
            has_colon = 1;
            break;
        }
    }

    // 2. 提取冒号前的运动状态
    uint8_t state_len = has_colon ? colon_pos : Len;
    if (state_len > sizeof(cmd_state) - 1)
        state_len = sizeof(cmd_state) - 1;
    for (i = 0; i < state_len; i++)
    {
        cmd_state[i] = Data[i];
    }

    // 字符串 → 枚举
    cmd->state = Parse_State(cmd_state);

    // 3. 提取冒号后的数据并存入cmd->data[]
    if (has_colon && colon_pos < Len - 1)
    {
        uint8_t info_start = colon_pos + 1;
        uint8_t info_len = Len - info_start;
        if (info_len > sizeof(cmd_info) - 1)
            info_len = sizeof(cmd_info) - 1;
        for (i = 0; i < info_len; i++)
        {
            cmd_info[i] = Data[info_start + i];
        }

        // 按逗号分割, 存入cmd->data[]
        uint8_t idx = 0;
        char *token = strtok(cmd_info, ",");
        while (token != NULL && idx < 20)
        {
            cmd->data[idx++] = atof(token);
            token = strtok(NULL, ",");
        }
        cmd->data_count = idx;
    }
}

//pc的命令不断解析和跑，解析结果通过cmd_out传出
//返回1=有新命令被解析, 0=无新数据, cmd_out未更新
uint8_t usb_pc_run(pc_command_t *cmd_out)
{
    if (cmd_out == NULL) return 0;

    // 轮询处理接收的数据（不阻塞）
    if (app_rx_wp != app_rx_rp)
    {
        USB_Parse_Data(
            cmd_out,
            app_rx_buf[app_rx_rp],
            app_rx_len[app_rx_rp]
        );

        // 移动读指针
        app_rx_rp = (app_rx_rp + 1) % APP_RX_BUF_NUM;
        return 1;
    }
    return 0;
}

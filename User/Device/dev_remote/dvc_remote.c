#include "dvc_remote.h"

#define SBUS_START 0x0F
#define SBUS_END 0x00
#define Rx_buf_size 64

/******遥控器数据缓冲区********/
uint8_t Rx_buf[Rx_buf_size] = {0};

/******定义FS-i6X遥控器数据结构体变量******/
Remote_control_struct Remote_control_FS;

void Remote_callback(uint8_t *data, uint16_t size)
{
    if (Rx_buf[0] == SBUS_START && Rx_buf[24] == SBUS_END)
    {
        /********SBUS协议解码 - FS-i6X遥控器**********/
        Remote_control_FS.Right_X = (uint16_t)((Rx_buf[1] | Rx_buf[2] << 8) & 0x07FF);
        Remote_control_FS.Right_Y = (uint16_t)((Rx_buf[2] >> 3 | Rx_buf[3] << 5) & 0x07FF);
        Remote_control_FS.Left_Y  = (uint16_t)((Rx_buf[3] >> 6 | Rx_buf[4] << 2 | Rx_buf[5] << 10) & 0x07FF);
        Remote_control_FS.Left_X  = (uint16_t)((Rx_buf[5] >> 1 | Rx_buf[6] << 7) & 0x07FF);
        Remote_control_FS.VRA     = (uint16_t)((Rx_buf[6] >> 4 | Rx_buf[7] << 4) & 0x07FF);
        Remote_control_FS.VRB     = (uint16_t)((Rx_buf[7] >> 7 | Rx_buf[8] << 1 | Rx_buf[9] << 9) & 0x07FF);
        Remote_control_FS.SWA     = (uint16_t)((Rx_buf[9] >> 2 | Rx_buf[10] << 6) & 0x07FF);
        Remote_control_FS.SWB     = (uint16_t)((Rx_buf[10] >> 5 | Rx_buf[11] << 3) & 0x07FF);
        Remote_control_FS.SWC     = (uint16_t)((Rx_buf[12] | Rx_buf[13] << 8) & 0x07FF);
        Remote_control_FS.SWD     = (uint16_t)((Rx_buf[13] >> 3 | Rx_buf[14] << 5) & 0x07FF);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&SBUS_UART, Rx_buf, Rx_buf_size);
}
#ifndef DVC_REMOTE_H
#define DVC_REMOTE_H

#include "main.h"
#include "drv_usart.h"
#include "dev_pc.h"

#define SBUS_UART huart3

/****FS-i6X遥控器数据结构体****/
typedef struct{       //低位~中值~高位
    uint16_t Right_X;  //286~1024~1746
    uint16_t Right_Y;  //240~1024~1678
    uint16_t Left_Y;   //303~1024~1765
    uint16_t Left_X;   //248~1024~1700
    uint16_t SWA;      //240~1807
    uint16_t SWB;      //240~1807
    uint16_t SWC;      //240~1024~1807
    uint16_t SWD;      //240~1807
    uint16_t VRA;      //298~1024~1805
    uint16_t VRB;      //240~1807
}Remote_control_struct;

extern uint8_t Rx_buf[64];

extern Remote_control_struct Remote_control_FS;
//函数定义
void Remote_Init(void);
void Remote_callback(uint8_t *data, uint16_t size);

#endif //DVC_REMOTE_H
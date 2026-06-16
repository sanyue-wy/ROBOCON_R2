
#ifndef DRV_CAN_H
#define DRV_CAN_H

#include "main.h"

//-----------------------------------------------------------------
// 如果启用CAN功能
#ifdef HAL_CAN_MODULE_ENABLED
#include "can.h"

typedef struct
{
    CAN_HandleTypeDef *hcan;
    void (*CAN_Callback)(CAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf);

} CAN_Interrupt_t;

// 把can的buf中的内容发送出去, 长度为8
// 返回值: 0=成功, 1=参数错误, 2=发送失败
uint8_t CAN_Transmit(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Buf);
/**
 * 联接CAN中断源和中断回调函数
 * 应输入CAN源, 回调函数个数以及回调函数的地址
 * 联接后自动开启中断
 */
void AttachInterrupt_CAN(CAN_HandleTypeDef *hcan, void (*CAN_Callback)(CAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf),uint32_t FilterBank);
#endif /* HAL_CAN_MODULE_ENABLED */



#endif //DRV_CAN_H
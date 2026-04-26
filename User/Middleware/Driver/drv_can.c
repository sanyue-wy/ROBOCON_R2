#include "drv_can.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

//----------------------BSP_CAN----------------------//
#ifdef HAL_CAN_MODULE_ENABLED

static CAN_TxHeaderTypeDef CAN_TxHeader;
static uint32_t CAN_TxMailbox = 0;

void CAN_Transmit(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Buf)
{
    if ((Buf != NULL))
    {
        CAN_TxHeader.StdId = ID;         /* 指定标准标识符，该值在0x00-0x7FF */
        CAN_TxHeader.IDE = CAN_ID_STD;   /* 指定将要传输消息的标识符类型 */
        CAN_TxHeader.RTR = CAN_RTR_DATA; /* 指定消息传输帧类型 */
        CAN_TxHeader.DLC = 8;            /* 指定将要传输的帧长度 */

        HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, Buf, &CAN_TxMailbox);
    }
}
#endif /* HAL_CAN_MODULE_ENABLED */


//--------------------------------------------------------------------------------------------------------------------
// CAN Interrupt
#ifdef HAL_CAN_MODULE_ENABLED

static CAN_RxHeaderTypeDef CAN_RxHeader;
static uint8_t CAN_RxBuf[8];

static uint8_t CAN_Function_Count = 0;      // 函数数量
static CAN_Interrupt_t *CAN_ItSource_Array; // CAN中断回调函数结构体数组指针

// 联接CAN中断源和中断回调函数
void AttachInterrupt_CAN(CAN_HandleTypeDef *hcan, void (*CAN_Callback)(CAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf))
{
    CAN_ItSource_Array = (CAN_Interrupt_t *)realloc(CAN_ItSource_Array, (CAN_Function_Count + 1) * sizeof(CAN_Interrupt_t));
    CAN_ItSource_Array[CAN_Function_Count].hcan = hcan;
    CAN_ItSource_Array[CAN_Function_Count].CAN_Callback = CAN_Callback;
    CAN_Function_Count++;

    CAN_FilterTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.FilterActivation = ENABLE;
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure.FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.FilterBank = 0;
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(hcan, &CAN_FilterInitStructure);
    HAL_CAN_Start(hcan);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// CAN中断回调
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN_RxBuf);

    for (uint8_t i = 0; i < CAN_Function_Count; i++)
    {
        if (CAN_ItSource_Array[i].hcan == hcan)
        {
            CAN_ItSource_Array[i].CAN_Callback(&CAN_RxHeader, CAN_RxBuf);
        }
    }
}
#endif /* HAL_CAN_MODULE_ENABLED */
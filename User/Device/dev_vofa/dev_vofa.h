
#ifndef DVC_VOFA_H
#define DVC_VOFA_H


#include "main.h"
#include "drv_usart.h"
#include <stdlib.h>
//FireWater  0
//JustFloat  1
#define VOFA_DATA_FORMAT 1
//调试模式 1
//不调试模式 0
#define debugmode 1

#define commandlength 200
#define framehead '='
#define frametail '!'

extern uint8_t DataBuff[commandlength];//指令内容

typedef struct{
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float position_kp;
    float position_ki;
    float position_kd;
    float speed_target;

    float angle_target;
}pid_para_t;


void uart_pid_to_pid_update(float * kp,float * ki,float * kd,float * target,uint8_t mode);
float Get_Data(void);
void USART_PID_Adjust(uint8_t Motor_n);
void vofa_FloatSend(float * date,uint16_t length);
void Vofa_Callback(uint8_t *data, uint16_t size);


void Float_to_Byte(float f,unsigned char byte[]);

void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);


#endif //DVC_VOFA_H
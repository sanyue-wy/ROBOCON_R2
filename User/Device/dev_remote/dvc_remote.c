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
    if (size < 25)
        goto rearm;

    if (data[0] == SBUS_START && data[24] == SBUS_END)
    {
        /********SBUS协议解码 - FS-i6X遥控器**********/
        Remote_control_FS.Right_X = (uint16_t)((data[1] | data[2] << 8) & 0x07FF);
        Remote_control_FS.Right_Y = (uint16_t)((data[2] >> 3 | data[3] << 5) & 0x07FF);
        Remote_control_FS.Left_Y  = (uint16_t)((data[3] >> 6 | data[4] << 2 | data[5] << 10) & 0x07FF);
        Remote_control_FS.Left_X  = (uint16_t)((data[5] >> 1 | data[6] << 7) & 0x07FF);
        Remote_control_FS.VRA     = (uint16_t)((data[6] >> 4 | data[7] << 4) & 0x07FF);
        Remote_control_FS.VRB     = (uint16_t)((data[7] >> 7 | data[8] << 1 | data[9] << 9) & 0x07FF);
        Remote_control_FS.SWA     = (uint16_t)((data[9] >> 2 | data[10] << 6) & 0x07FF);
        Remote_control_FS.SWB     = (uint16_t)((data[10] >> 5 | data[11] << 3) & 0x07FF);
        Remote_control_FS.SWC     = (uint16_t)((data[12] | data[13] << 8) & 0x07FF);
        Remote_control_FS.SWD     = (uint16_t)((data[13] >> 3 | data[14] << 5) & 0x07FF);
    }
rearm:
    HAL_UARTEx_ReceiveToIdle_DMA(&SBUS_UART, Rx_buf, Rx_buf_size);
}
static bool remote_ansys(pc_command_t *out)
{

    //遥感控制运动
    if (Remote_control_FS.SWA > 1024)
    {
        if (Remote_control_FS.Right_Y > 1024)
        {
            out->motion = CAR_FORWARD;
            out->data = Remote_control_FS.Right_Y-1024;
        }
        else if (Remote_control_FS.Right_Y < 1024)
        {
            out->motion = CAR_BACKWARD;
            out->data = Remote_control_FS.Right_Y;
        }
        else
            out->motion = CAR_STOP;
        if (Remote_control_FS.Right_X > 1024)
        {
            out->motion = CAR_TRANSLATE_RIGHT;
            out->data = Remote_control_FS.Right_X-1024;
        }
        else if (Remote_control_FS.Right_X < 1024)
        {
            out->motion = CAR_TRANSLATE_LEFT;
            out->data = Remote_control_FS.Right_X;
        }
        else
            out->motion = CAR_STOP;
        if (Remote_control_FS.Left_X > 1024)
        {
            out->motion = CAR_TURN_RIGHT;
            out->data = Remote_control_FS.Left_X-1024;
        }
        else if (Remote_control_FS.Left_X < 1024)
        {
            out->motion = CAR_TURN_LEFT;
            out->data = Remote_control_FS.Left_X;
        }
        else
            out->motion = CAR_STOP;

        //夹取端头的程序
        if ( Remote_control_FS.SWB>1700)
        {
            if ( 200<Remote_control_FS.Left_Y&&Remote_control_FS.Left_Y<700)
            {
                out->motion =CAR_FINGER;
                out->data =44444;
            }
            if ( 700<Remote_control_FS.Left_Y&&Remote_control_FS.Left_Y<1200)
            {
                out->motion =CAR_FINGER;
                out->data =55555;
                HAL_Delay(500);
                out->motion=CAR_FINGER_WRIST;
                out->data=180;
            }
            if ( 1500<Remote_control_FS.Left_Y&&Remote_control_FS.Left_Y<1800)
            {
                out->motion =CAR_FINGER;
                out->data =44444;
                HAL_Delay(500);
                out->motion=CAR_FINGER_WRIST;
                out->data=90;
            }
        }

    }



    else
        out->motion = CAR_STOP;
}
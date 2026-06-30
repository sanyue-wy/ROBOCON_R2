#include "dvc_remote.h"

#define SBUS_START 0x0F
#define SBUS_END 0x00
#define Rx_buf_size 64

/******遥控器数据缓冲区********/
uint8_t Rx_buf[Rx_buf_size] = {0};

/******定义FS-i6X遥控器数据结构体变量******/
Remote_control_struct Remote_control_FS;
void Remote_Init(void)
{

    AttachInterrupt_UART_DMA(&huart3,Rx_buf,64,Remote_callback);
}


void Remote_callback(uint8_t *data, uint16_t size)
{
    if (size < 25)
     HAL_UARTEx_ReceiveToIdle_DMA(&SBUS_UART, Rx_buf, Rx_buf_size);
        return;

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
}


/**
 * @brief 遥控器数据解析 - 根据remote_rule.md规则
 * @param out 输出命令结构体
 * @return true=有效命令, false=无有效输入
 *
 * 遥控器映射规则:
 * SWA - 总使能（必须拨上才工作）
 * Right_Y - 前后    Right_X - 横移    Left_X - 旋转
 *
 * SWC低(240): Left_Y控制夹爪
 *   最下面(303): 夹爪松开，舵机默认
 *   中间(800-1200): 夹紧
 *   最上面(1765): 旋转+90度
 *
 * SWC中(1024): Left_Y控制升降台, SWB控制车子升降
 * SWC高(1807): Left_Y→DOF1, VRA→DOF2, VRB→DOF3, SWD→吸盘
 */
bool remote_ansys(pc_command_t *out)
{
    static uint16_t pre_left_y = 0;

    // SWA总使能检查 - 必须拨上才工作
    if (Remote_control_FS.SWA < 1024)
    {
        out->motion = CAR_STOP;
        out->data = 0;
        return true;
    }

    // ========== 运动控制（优先级最高） ==========
    // Right_Y → 前后 (速度0~10000, 映射到0~1.592m/s)
    else if (Remote_control_FS.Right_Y > 1100)
    {
        out->motion = CAR_FORWARD;
        out->data = map_speed(Remote_control_FS.Right_Y, 1024);
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }
    else if (Remote_control_FS.Right_Y < 900)
    {
        out->motion = CAR_BACKWARD;
        out->data = map_speed(Remote_control_FS.Right_Y, 1024);
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }

    // Right_X → 横移 (速度0~10000, 映射到0~1.592m/s)
    else if (Remote_control_FS.Right_X > 1100)
    {
        out->motion = CAR_TRANSLATE_RIGHT;
        out->data = map_speed(Remote_control_FS.Right_X, 1024);
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }
    else if (Remote_control_FS.Right_X < 900)
    {
        out->motion = CAR_TRANSLATE_LEFT;
        out->data = map_speed(Remote_control_FS.Right_X, 1024);
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }

    // Left_X → 旋转 (速度0~10000, 映射到0~2.49rad/s)
   else if (Remote_control_FS.Left_X > 1100)
    {
        out->motion = CAR_TURN_RIGHT;
        out->data = map_speed(Remote_control_FS.Left_X, 1024);
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }
    else if (Remote_control_FS.Left_X < 900)
    {
        out->motion = CAR_TURN_LEFT;
        out->data = map_speed(Remote_control_FS.Left_X, 1024);
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }

    // ========== SWC模式切换 ==========
    // SWC低（~240）：Left_Y控制夹爪
    else if (Remote_control_FS.SWC < 400)
    {
        // Left_Y最下面（~303）：夹爪松开，舵机默认
        if (Remote_control_FS.Left_Y < 400)
        {
            out->motion = CAR_FINGER;
            out->data = 44444; // 松开
        }
        // Left_Y中间（~800-1200）：夹爪夹紧
        else if (Remote_control_FS.Left_Y > 800 && Remote_control_FS.Left_Y < 1200)
        {
            out->motion = CAR_FINGER;
            out->data = 55555; // 夹紧
        }
        // Left_Y最上面（~1765）：旋转+90度
        else if (Remote_control_FS.Left_Y > 1500)
        {
            out->motion = CAR_FINGER_WRIST;
            out->data = map_angle_0_180(Remote_control_FS.Left_Y);
        }
        // Left_Y从最上面回到中间：舵机恢复默认
        else if (Remote_control_FS.Left_Y > 1200 && Remote_control_FS.Left_Y < 1500
                 && pre_left_y > 1500)
        {
             out->motion = CAR_FINGER;
            out->data = 44444; // 松开
        }
        else
        {
            out->motion = CAR_STOP;
            out->data = 0;
        }
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }
    // SWC中（~1024）：Left_Y控制升降台，SWB控制车子升降
    else if (Remote_control_FS.SWC > 900 && Remote_control_FS.SWC < 1200)
    {
        // SWB高（~1807）：车子抬升
        if (Remote_control_FS.SWB > 1500)
        {
            out->motion = CAR_UP;
            out->data = 1;
        }
        // SWB低（~240）：车子下降
        else if (Remote_control_FS.SWB < 500)
        {
            out->motion = CAR_DOWN;
            out->data = 1;
        }
        // Left_Y控制升降台高度 (距离mm)
        else if (Remote_control_FS.Left_Y > 1100)
        {
            out->motion = CAR_TABLE_UP;
            out->data = map_speed(Remote_control_FS.Left_Y, 1024);
        }
        else if (Remote_control_FS.Left_Y < 900)
        {
            out->motion = CAR_TABLE_DOWN;
            out->data = map_speed(Remote_control_FS.Left_Y, 1024);
        }
        else
        {
            out->motion = CAR_STOP;
            out->data = 0;
        }
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }
    // SWC高（~1807）：Left_Y→DOF1, VRA→DOF2, VRB→DOF3, SWD→吸盘
    else if (Remote_control_FS.SWC > 1700)
    {
        // SWD高（~1807）：吸盘吸住
        if (Remote_control_FS.SWD > 1500)
        {
            out->motion = CAR_SUCK;
            out->data = 55555; // 吸住
        }
        // SWD低（~240）：吸盘不吸
        else if (Remote_control_FS.SWD < 500)
        {
            out->motion = CAR_SUCK;
            out->data = 44444; // 放下
        }
        // Left_Y控制DOF1角度 (0~90度, 映射到-45~45度)
        // 向上推: data=0~90 (正方向), 向下推: data=90~180 (负方向)
        else if (Remote_control_FS.Left_Y > 1100)
        {
            out->motion = CAR_DOFF;
            out->data = map_angle_0_90(Remote_control_FS.Left_Y);
        }
        else if (Remote_control_FS.Left_Y < 900)
        {
            out->motion = CAR_DOFF;
            out->data = 90 + map_angle_0_90(Remote_control_FS.Left_Y);
        }
        // VRA控制DOF2
        else if (Remote_control_FS.VRA > 1100)
        {
            out->motion = CAR_DOFS;
            out->data = map_angle_0_90(Remote_control_FS.VRA);
        }
        else if (Remote_control_FS.VRA < 900)
        {
            out->motion = CAR_DOFS;
            out->data = map_angle_0_90(Remote_control_FS.VRA);
        }
        // VRB控制DOF3
        else if (Remote_control_FS.VRB > 1100)
        {
            out->motion = CAR_DOFT;
            out->data = map_angle_0_90(Remote_control_FS.VRB);
        }
        else if (Remote_control_FS.VRB < 900)
        {
            out->motion = CAR_DOFT;
            out->data = map_angle_0_90(Remote_control_FS.VRB);
        }
        else
        {
            out->motion = CAR_STOP;
            out->data = 0;
        }
        pre_left_y = Remote_control_FS.Left_Y;
        return true;
    }

    // 无有效输入（摇杆居中）
    out->motion = CAR_STOP;
    out->data = 0;
    pre_left_y = Remote_control_FS.Left_Y;
    return true;
}

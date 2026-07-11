//
// Created by wy on 2026/5/18.
//

#include "dev_pc.h"
#include "drv_usb.h"
#include <string.h>
#include <ctype.h>

// 字节流环形缓冲区大小（建议 2048，足够缓冲多帧）
#define RX_RING_SIZE    2048

// 命令队列深度（可缓存多个已解析的命令）
#define CMD_QUEUE_SIZE  4

// 帧长度固定：1 字节头 + 4 字节运动 + 5 字节数据 = 10 字节
#define FRAME_LEN       10

static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_wr = 0;   // 写入索引
static volatile uint16_t rx_rd = 0;   // 读取索引

pc_command_t current_command={0};

// 命令队列（存储解析好的完整命令）
static pc_command_t cmd_queue[CMD_QUEUE_SIZE];
static volatile uint8_t cmd_wp = 0;
static volatile uint8_t cmd_rp = 0;

// 将 4 字节状态字符串（小写）转换为枚举
static car_motion_t parse_motion(const char *str)
{
    if (strncmp(str, "stop", 4) == 0) return CAR_STOP;
    if (strncmp(str, "fwrd", 4) == 0) return CAR_FORWARD;    //速度数据给到0到10000，映射到0到1.592m/s
    if (strncmp(str, "bwrd", 4) == 0) return CAR_BACKWARD;
    if (strncmp(str, "ltrn", 4) == 0) return CAR_TURN_LEFT;
    if (strncmp(str, "rtrn", 4) == 0) return CAR_TURN_RIGHT;  //速度数据给到0到10000,映射到0到2.49rad/s
    if (strncmp(str, "ltrl", 4) == 0) return CAR_TRANSLATE_LEFT;
    if (strncmp(str, "rtrl", 4) == 0) return CAR_TRANSLATE_RIGHT;
    if (strncmp(str, "upwd", 4) == 0) return CAR_UP;        //44444为爬楼状态，55555为放物块的
    if (strncmp(str, "dwnd", 4) == 0) return CAR_DOWN;      //44444为爬楼状态，55555为放物块的
    if (strncmp(str, "airb", 4) == 0) return CAR_AIR_BREAK;     // 气路总阀，55555为开启，44444为关闭
    if (strncmp(str, "dofs", 4) == 0) return CAR_DOFS;     //
    if (strncmp(str, "doft", 4) == 0) return CAR_DOFT;     //0——360映射到-180到180
    if (strncmp(str, "suck", 4) == 0) return CAR_SUCK;     //吸盘吸取物块  后面五位数据给55555为吸住，给44444为放下
    if (strncmp(str, "taup", 4) == 0) return CAR_TABLE_UP;    //升降台  后面数据为上升距离 单位mm
    if (strncmp(str, "tadw", 4) == 0) return CAR_TABLE_DOWN;
    if (strncmp(str, "fing", 4) == 0) return CAR_FINGER;  //气动拇指夹取武器 后面五位数据给55555为夹取，给44444为放下
    if (strncmp(str, "fwrs", 4) == 0) return CAR_FINGER_WRIST; //气动手指腕部舵机旋转角度，后面数据给0到220，映射到-110到110度,正的手指向上
    return CAR_STOP;
}

// 将 5 位 ASCII 数字字符串转换为整数 (0~65535)
static uint16_t parse_data(const char *str)
{
    uint16_t val = 0;
    for (int i = 0; i < 5; i++) {
        char c = str[i];
        if (c < '0' || c > '9') break;
        val = val * 10 + (c - '0');
    }
    return val;
}

// 从字节流环形缓冲区中提取一帧（如果有），填充 out，返回 true
static bool try_extract_frame(pc_command_t *out)
{
    // 计算可读字节数
    uint16_t available;
    if (rx_wr >= rx_rd)
        available = rx_wr - rx_rd;
    else
        available = RX_RING_SIZE - rx_rd + rx_wr;

    if (available < FRAME_LEN) return false;   // 不够一帧

    // 搜索帧头 'M'
    uint16_t search_pos = rx_rd;
    uint16_t found_pos = RX_RING_SIZE; // 无效值
    uint16_t tmp_rd = rx_rd;
    uint16_t tmp_wr = rx_wr;

    for (uint16_t i = 0; i <= available - FRAME_LEN; i++) {
        uint16_t pos = (tmp_rd + i) % RX_RING_SIZE;
        if (rx_ring[pos] == PC_FRAME_HEADER) {
            found_pos = pos;
            break;
        }
    }

    if (found_pos == RX_RING_SIZE) return false;   // 未找到帧头

    // 提取 10 字节帧数据
    uint8_t frame[FRAME_LEN];
    for (uint16_t i = 0; i < FRAME_LEN; i++) {
        uint16_t idx = (found_pos + i) % RX_RING_SIZE;
        frame[i] = rx_ring[idx];
    }

    // 解析运动状态（frame[1]~frame[4]）
    char motion_str[5] = {0};
    for (int i = 0; i < 4; i++) {
        motion_str[i] = tolower(frame[1 + i]);
    }
    // 解析数据（frame[5]~frame[9]）
    char data_str[6] = {0};
    memcpy(data_str, &frame[5], 5);

    out->motion = parse_motion(motion_str);
    out->data   = parse_data(data_str);
    out->valid  = true;

    // 移动读指针，跳过已处理的帧
    rx_rd = (found_pos + FRAME_LEN) % RX_RING_SIZE;
    return true;
}

// USB 接收回调（在中断中运行）
void usb_receive_callback(uint8_t *Buffer, uint16_t Length)
{
    if (Length == 0 || Length > USB_BUFFER_SIZE) return;

    for (uint16_t i = 0; i < Length; i++) {
        uint16_t next_wr = (rx_wr + 1) % RX_RING_SIZE;
        if (next_wr == rx_rd) {
            // 环形缓冲区满：丢弃新数据（可记录错误标志）
            break;
        }
        rx_ring[rx_wr] = Buffer[i];
        rx_wr = next_wr;
    }
}

// 初始化 USB 并注册回调
void usb_pc_init(void)
{
    rx_wr = 0;
    rx_rd = 0;
    cmd_wp = 0;
    cmd_rp = 0;
    USB_Init(usb_receive_callback);
}

// 主循环中调用，获取一个命令。返回 true 表示有有效命令
bool usb_pc_run(pc_command_t *out_cmd)
{
    if (out_cmd == NULL) return false;

    // 先检查命令队列是否有已解析的命令
    if (cmd_wp != cmd_rp) {
        *out_cmd = cmd_queue[cmd_rp];
        cmd_rp = (cmd_rp + 1) % CMD_QUEUE_SIZE;
        return true;
    }

    // 尝试从字节流中提取一帧
    pc_command_t new_cmd;
    if (try_extract_frame(&new_cmd)) {
        // 可以立即返回，也可以放入队列（这里直接返回）
        *out_cmd = new_cmd;
        return true;
    }

    return false;
}

// 发送数据到 PC
void Transmit_to_PC(uint8_t *Data, uint16_t Length)
{
    if (Data == NULL || Length == 0) return;
    USB_Transmit_Data(Data, Length);
}
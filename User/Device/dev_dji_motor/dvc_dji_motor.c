#include "dvc_dji_motor.h"
#include "drv_usart.h"

#define M3508_DEC 19 // 3508减速比
#define M2006_DEC 36 // 2006减速比

#define MAX_CURRENT 16384  // 最大电流
#define MIN_CURRENT -16384 // 最小电流

#define CAN_BUS_NUM 2  // CAN总线数量

DJ_Motor_t DJ_Motor3508[3];

/* 电机ID地址 */
enum CAN_Motor_ID
{
    DJ_H_ID = 0x1FF,
    DJ_L_ID = 0x200,
    DJ_M1_ID = 0x201,
    DJ_M2_ID = 0x202,
    DJ_M3_ID = 0x203,
    DJ_M4_ID = 0x204,
    DJ_M5_ID = 0x205,
    DJ_M6_ID = 0x206,
    DJ_M7_ID = 0x207,
    DJ_M8_ID = 0x208,
};

volatile static uint8_t motorCount[CAN_BUS_NUM]; // 每条CAN总线的电机数量
static DJ_Motor_t *Motors[CAN_BUS_NUM][8];       // 按CAN总线分组的电机指针数组

/* 根据CAN句柄获取总线索引: hcan1->0, hcan2->1 */
static inline uint8_t get_bus_index(CAN_HandleTypeDef *hcan)
{
    return (hcan == &hcan1) ? 0 : 1;
}

/* CAN句柄查找表，按总线索引 */
static CAN_HandleTypeDef * const hcan_handles[CAN_BUS_NUM] = { &hcan1, &hcan2 };

#ifdef HAL_CAN_MODULE_ENABLED
/* CAN中断回调函数 */
void DJ_CAN_Callback(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf)
{
    switch (pHeader->StdId)
    {
    case DJ_M1_ID:
    case DJ_M2_ID:
    case DJ_M3_ID:
    case DJ_M4_ID:
    case DJ_M5_ID:
    case DJ_M6_ID:
    case DJ_M7_ID:
    case DJ_M8_ID:
        {
            uint8_t i = pHeader->StdId - DJ_M1_ID;
            uint8_t bus = get_bus_index(hcan);

            // 只在消息来源总线上查找电机
            DJ_Motor_t *motor = NULL;
            if (Motors[bus][i] != NULL && Motors[bus][i]->initialized)
            {
                motor = Motors[bus][i];
            }
            if (motor == NULL)
                break;

            motor->last_angle = motor->angle;
            motor->angle = (uint16_t)(pBuf[0] << 8 | pBuf[1]);
            motor->speed = (uint16_t)(pBuf[2] << 8 | pBuf[3]);
            motor->current = (uint16_t)(pBuf[4] << 8 | pBuf[5]);

            if (motor->angle - motor->last_angle > 4096)
            {
                motor->round_count--;
            }
            else if (motor->angle - motor->last_angle < -4096)
            {
                motor->round_count++;
            }
            motor->total_angle = (motor->round_count + (motor->angle - motor->offset_angle) / 8192.0f) * 360.0f / (float)motor->dec;
        }
    }
}
#endif


/**
 * 电机参数初始化
 * 不可以重复初始化同一个电机
 */
void DJ_Init(DJ_Motor_t *motor, uint8_t Motor_ID, DJ_MotorType_e Motor_Type, DJ_ControllMethod_e method, CAN_HandleTypeDef *hcan)
{
#if PID

    if (method == PID_METHOD)
    {
        if (Motor_Type == M2006)
        {
            motor->dec = M2006_DEC;
            // 速度PID初始化
            PID_Init(&motor->PID_Speed, 5.0f, 0.5f, 0.0f, 16000.0f, 1000.0f);
            // 角度PID初始化
            TDPID_Init(&motor->PID_Angle, 150.0f, 0.5f, 10.0f, 5000.0f, 50.0f, 0.003f, 10000);
            PID_Init(&motor->PID_SpeedOfAngle, 5.0f, 0.0f, 0.0f, 16000.0f, 500.0f);
        }
        else if (Motor_Type == M3508)
        {
            motor->dec = M3508_DEC;
            // 速度PID初始化
            PID_Init(&motor->PID_Speed, 5.0f, 0.5f, 0.0f, 16000.0f, 1000.0f);
            // 角度PID初始化
            TDPID_Init(&motor->PID_Angle, 100.0f, 0.3f, 5.0f, 9000.0f, 50.0f, 0.003f, 10000);
            PID_Init(&motor->PID_SpeedOfAngle, 5.0f, 0.0f, 0.0f, 16000.0f, 200.0f);
        }
    }
#endif

#if IMPEDANCE
    if (method == IMPEDANCE_METHOD)
    {
        if (Motor_Type == M2006)
        {
            motor->dec = M2006_DEC;
        }
        else if (Motor_Type == M3508)
        {
            motor->dec = M3508_DEC;
        }
    }
#endif

    uint8_t bus = get_bus_index(hcan);
    motor->method = method;
    motor->initialized = 1;
    motor->can_bus = hcan;
    motorCount[bus]++;
    motor->ID = DJ_L_ID + Motor_ID;
    motor->setCurrent = 0;
    motor->offset_angle = 0;
    motor->round_count = 0;
    Motors[bus][Motor_ID - 1] = motor;
    motor->offset_angle = motor->last_angle; // 获取偏移角度
}


//--------------------------------------------------------------------------
// 大疆电机控制
// 需要循环执行此函数
void DJ_MotorRun(void)
{
    uint8_t data[8];

    /* 按CAN总线分组：计算PID并发送电流 */
    for (uint8_t bus = 0; bus < CAN_BUS_NUM; bus++)
    {
        /* 第一步：计算该总线上所有电机的PID */
        for (uint8_t i = 0; i < 8; i++)
        {
            // 跳过未初始化或未注册的电机
            if (Motors[bus][i] == NULL || !Motors[bus][i]->initialized)
                continue;

#if PID
            if (Motors[bus][i]->method == PID_METHOD)
            {
                if (Motors[bus][i]->mode == ANGLE_MODE || Motors[bus][i]->mode == ANGLEINC_MODE)
                {
                    // 串级PID控制
                    TDPID_Calc(&Motors[bus][i]->PID_Angle, (Motors[bus][i]->total_angle), Motors[bus][i]->setAngle);
                    PID_Calc(&Motors[bus][i]->PID_SpeedOfAngle, (float)(Motors[bus][i]->speed), Motors[bus][i]->PID_Angle.out);
                    Motors[bus][i]->setCurrent = (int16_t)Motors[bus][i]->PID_SpeedOfAngle.out;
                }
                else if (Motors[bus][i]->mode == SPEED_MODE)
                {
                    PID_Calc(&Motors[bus][i]->PID_Speed, (float)(Motors[bus][i]->speed), Motors[bus][i]->setSpeed);
                    Motors[bus][i]->setCurrent = (int16_t)Motors[bus][i]->PID_Speed.out;
                }
            }
#endif

#if IMPEDANCE
            if (Motors[bus][i]->method == IMPEDANCE_METHOD)
            {
                Motors[bus][i]->imp.dp = (float)(Motors[bus][i]->setAngle - Motors[bus][i]->total_angle);
                Motors[bus][i]->imp.dv = (float)(Motors[bus][i]->setSpeed - Motors[bus][i]->speed);
                Impedance_Calc(&Motors[bus][i]->imp);
                Motors[bus][i]->setCurrent = (int16_t)(LIHT_MIN_MAX(Motors[bus][i]->imp.out, MIN_CURRENT, MAX_CURRENT));
            }
#endif
        }

        /* 第二步：发送该总线的电流指令 — 低4个电机 (ID 1-4) */
        for (uint8_t i = 0; i < 4; i++)
        {
            int16_t current = 0;
            if (Motors[bus][i] != NULL && Motors[bus][i]->initialized)
                current = Motors[bus][i]->setCurrent;
            data[i * 2]     = current >> 8;
            data[i * 2 + 1] = current;
        }
#ifdef HAL_CAN_MODULE_ENABLED
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_handles[bus]) > 0)
        {
            CAN_Transmit(hcan_handles[bus], DJ_L_ID, data);
        }
#endif
#ifdef HAL_FDCAN_MODULE_ENABLED
        while (HAL_FDCAN_GetTxFifoFreeLevel(hcan_handles[bus]) == 0)
            continue;
        FDCAN_Transmit(hcan_handles[bus], DJ_L_ID, data);
#endif

        /* 发送该总线的电流指令 — 高4个电机 (ID 5-8) */
        for (uint8_t i = 4; i < 8; i++)
        {
            int16_t current = 0;
            if (Motors[bus][i] != NULL && Motors[bus][i]->initialized)
                current = Motors[bus][i]->setCurrent;
            data[(i - 4) * 2]     = current >> 8;
            data[(i - 4) * 2 + 1] = current;
        }
#ifdef HAL_CAN_MODULE_ENABLED
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan_handles[bus]) > 0)
        {
            CAN_Transmit(hcan_handles[bus], DJ_H_ID, data);
        }
#endif
#ifdef HAL_FDCAN_MODULE_ENABLED
        while (HAL_FDCAN_GetTxFifoFreeLevel(hcan_handles[bus]) == 0)
            continue;
        FDCAN_Transmit(hcan_handles[bus], DJ_H_ID, data);
#endif
    }
}

/* 电机角度和圈数清零 */
inline void DJ_ClearAngle(DJ_Motor_t *motor)
{
    motor->total_angle = 0;
    motor->round_count = 0;
}

#if PID
/**
 * 设置电机速度
 * 设置一次即可
 * 单位:未减速前的r/min
 */
inline void DJ_SetSpeed(DJ_Motor_t *motor, float speed)
{
    motor->setSpeed = speed;
    motor->mode = SPEED_MODE;
}

/**
 * 设置电机角度
 * 设置一次即可
 * 单位:度
 */
inline void DJ_SetAngle(DJ_Motor_t *motor, float angle, float speed)
{
    motor->setAngle = angle;
    motor->PID_Angle.max_out = speed;
    motor->mode = ANGLE_MODE;
}

/**
 * 设置电机增量角度
 * 设置一次即可
 * 单位:度
 */
inline void DJ_SetAngleInc(DJ_Motor_t *motor, float angleInc)
{
    motor->setAngle = motor->total_angle + angleInc;
    motor->mode = ANGLEINC_MODE;
}
#endif /* PID */

#if IMPEDANCE
/**
 * 设置阻抗模式速度
 * 设置一次即可
 * 单位:度
 */
inline void DJ_SetImpSpeed(DJ_Motor_t *motor, float kd, float speed, int16_t current)
{
    motor->imp.Kp = 0;
    motor->imp.Kd = kd;
    motor->setAngle = 0;
    motor->setSpeed = speed;
    motor->setCurrent = current;
}

/**
 * 设置阻抗模式角度
 * 设置一次即可
 * 单位:度
 */
inline void DJ_SetImpAngle(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t t)
{
    motor->imp.Kp = kp;
    motor->imp.Kd = kd;
    motor->imp.t = t;
    motor->setAngle = angle;
    motor->setSpeed = 0;
}

/**
 * 设置阻抗模式增量角度
 * 设置一次即可
 * 单位:度
 */
inline void DJ_SetImpAngleInc(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t t)
{
    motor->imp.Kp = kp;
    motor->imp.Kd = kd;
    motor->imp.t = t;
    motor->setAngle = motor->total_angle + angle;
    motor->setSpeed = 0;
}

/* 设置阻尼模式 */
inline void DJ_SetImpDamp(DJ_Motor_t *motor, float kd)
{
    motor->imp.Kp = 0;
    motor->imp.Kd = kd;
    motor->imp.t = 0;
    motor->setAngle = 0;
    motor->setSpeed = 0;
}

/* 设置零力矩模式 */
inline void DJ_SetImpZeroTorque(DJ_Motor_t *motor)
{
    motor->imp.Kp = 0;
    motor->imp.Kd = 0;
    motor->imp.t = 150;
    motor->setAngle = 0;
    motor->setSpeed = 0;
}

#endif /* IMPEDANCE */

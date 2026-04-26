
#ifndef DVC_DJI_MOTOR_H
#define DVC_DJI_MOTOR_H


#ifdef __cplusplus
extern "C"
{
#endif

#include "drv_can.h"
#include "alg_pid.h"

#define PID 1
#define IMPEDANCE 0

    typedef enum
    {
        M3508 = 0,
        M2006 = 1,
    } DJ_MotorType_e; // 电机类型

    typedef enum
    {
        PID_METHOD = 0,       // PID控制
        IMPEDANCE_METHOD = 1, // 阻抗控制
    } DJ_ControllMethod_e;    // 电机控制方法

    /* 电机模式 */
    typedef enum
    {
        CURRENT_MODE = 0,
        SPEED_MODE = 1,
        ANGLE_MODE = 2,
        ANGLEINC_MODE = 3,
    } DJ_MotorMode_e; // 电机控制模式

    /* 电机反馈参数 */
    typedef struct
    {
        float setAngle;     // 设置角度
        float setSpeed;     // 设置速度
        int16_t setCurrent; // 设置电流

        uint16_t angle;      // CAN读取的角度
        int16_t speed;       // CAN读取的速度
        int16_t current;     // CAN读取的电流
        uint16_t last_angle; // 上一次读取的角度

        uint16_t ID;                // 电机的ID
        DJ_MotorType_e type;        // 电机类型
        DJ_MotorMode_e mode;        // 控制模式
        int16_t dec;                // 减速比
        DJ_ControllMethod_e method; // 控制方法
        uint16_t offset_angle;      // 电机初始偏移角度
        float total_angle;          // 电机总角度
        int16_t round_count;        // 电机圈数

#if PID
        PID_t PID_Speed;        // 电机速度环PID
        TDPID_t PID_Angle;      // 电机角度环PID
        PID_t PID_SpeedOfAngle; // 电机位置控制的速度环PID
#endif

#if IMPEDANCE
        Impedance_t imp;
#endif
    }  DJ_Motor_t;
    extern DJ_Motor_t DJ_Motor3508[3];
    extern DJ_Motor_t DJ_Motor2006[2];
    extern DJ_Motor_t chassis_Motor[4];

#ifdef HAL_CAN_MODULE_ENABLED
    void DJ_CAN_Callback(CAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf);
#endif

#ifdef HAL_FDCAN_MODULE_ENABLED
    void DJ_CAN_Callback(FDCAN_RxHeaderTypeDef *pHeader, uint8_t *pBuf);
#endif

    void DJ_Init(DJ_Motor_t *motor, uint8_t Motor_ID, DJ_MotorType_e Motor_Type, DJ_ControllMethod_e method);
    void DJ_MotorRun(void);
    void DJ_ClearAngle(DJ_Motor_t *motor);

#if PID
    void DJ_SetSpeed(DJ_Motor_t *motor, float speed);
    void DJ_SetAngle(DJ_Motor_t *motor, float angle, float speed);
    void DJ_SetAngleInc(DJ_Motor_t *motor, float angleInc);
#endif

#if IMPEDANCE
    void DJ_SetImpSpeed(DJ_Motor_t *motor, float kd, float speed, int16_t current);
    void DJ_SetImpAngle(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t current);
    void DJ_SetImpAngleInc(DJ_Motor_t *motor, float kp, float kd, float angle, int16_t current);
    void DJ_SetImpDamp(DJ_Motor_t *motor, float kd);
    void DJ_SetImpZeroTorque(DJ_Motor_t *motor);
#endif

#ifdef __cplusplus
}
#endif



#endif //DVC_DJI_MOTOR_H
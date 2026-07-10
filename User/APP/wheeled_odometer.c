/**
 * @file    wheeled_odometer.c
 * @brief   基于卡尔曼滤波的轮式里程计实现
 *
 * 融合数据源：
 *   - IMU加速度：INS.MotionAccel_n[X/Y]（来自ins_task）
 *   - 轮式编码器：chassis.ChassisMotors[i].speed（来自Chassis）
 *   - 航向角：INS.Yaw（来自ins_task）
 */

#include "wheeled_odometer.h"
#include "ins_task.h"
#include "Chassis.h"
#include "math.h"
#include "string.h"

/*--------------------------------------------------------------
 * 全局变量
 *--------------------------------------------------------------*/
Odometer_t Odometer;

/*--------------------------------------------------------------
 * 卡尔曼滤波器实例
 *--------------------------------------------------------------*/
static KalmanFilter_t KF_PosX;  // X轴位置-速度滤波器
static KalmanFilter_t KF_PosY;  // Y轴位置-速度滤波器

/*--------------------------------------------------------------
 * X轴滤波器矩阵数据
 *--------------------------------------------------------------*/
static float F_x[4] = {
    1.0f, KF_DT,   // [1, dt]
    0.0f, 1.0f     // [0, 1 ]
};

static float B_x[2] = {
    0.5f * KF_DT * KF_DT,  // 0.5*dt²
    KF_DT                    // dt
};

static float Q_x[4];  // 过程噪声矩阵（运行时计算）
static float P_x[4] = {1.0f, 0.0f, 0.0f, 1.0f};  // 初始协方差
static float H_x[2] = {0.0f, 1.0f};  // 观测矩阵：观测速度
static float R_x[1] = {ENCODER_NOISE * ENCODER_NOISE};  // 测量噪声
static float min_var_x[2] = {0.01f, 0.005f};  // 最小方差限制

/*--------------------------------------------------------------
 * Y轴滤波器矩阵数据（与X轴相同）
 *--------------------------------------------------------------*/
static float F_y[4];
static float B_y[2];
static float Q_y[4];
static float P_y[4];
static float H_y[2];
static float R_y[1];
static float min_var_y[2];

/*--------------------------------------------------------------
 * 内部函数
 *--------------------------------------------------------------*/

/**
 * @brief 计算过程噪声矩阵Q
 *        Q = [0.25*dt^4, 0.5*dt^3] * σ²
 *            [0.5*dt^3,   dt^2    ]
 */
static void Calc_Q_Matrix(float *Q, float dt, float noise)
{
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float noise2 = noise * noise;

    Q[0] = 0.25f * dt4 * noise2;
    Q[1] = 0.5f * dt3 * noise2;
    Q[2] = Q[1];
    Q[3] = dt2 * noise2;
}

/**
 * @brief 初始化滤波器矩阵
 */
static void Init_KF_Matrices(KalmanFilter_t *kf,
                              float *F, float *B, float *Q, float *P,
                              float *H, float *R, float *min_var,
                              uint8_t state_dim, uint8_t meas_dim)
{
    memcpy(kf->F_data, F, sizeof(float) * state_dim * state_dim);
    memcpy(kf->B_data, B, sizeof(float) * state_dim * 1);
    memcpy(kf->Q_data, Q, sizeof(float) * state_dim * state_dim);
    memcpy(kf->P_data, P, sizeof(float) * state_dim * state_dim);
    memcpy(kf->H_data, H, sizeof(float) * meas_dim * state_dim);
    memcpy(kf->R_data, R, sizeof(float) * meas_dim * meas_dim);
    memcpy(kf->StateMinVariance, min_var, sizeof(float) * state_dim);
}

/*--------------------------------------------------------------
 * 公开函数实现
 *--------------------------------------------------------------*/

void Odometer_Init(void)
{
    /* 清零里程计数据 */
    memset(&Odometer, 0, sizeof(Odometer_t));

    /* 计算过程噪声矩阵 */
    Calc_Q_Matrix(Q_x, KF_DT, ACCEL_NOISE);
    Calc_Q_Matrix(Q_y, KF_DT, ACCEL_NOISE);

    /* 复制矩阵到Y轴 */
    memcpy(F_y, F_x, sizeof(F_x));
    memcpy(B_y, B_x, sizeof(B_x));
    memcpy(Q_y, Q_x, sizeof(Q_x));
    memcpy(P_y, P_x, sizeof(P_x));
    memcpy(H_y, H_x, sizeof(H_x));
    memcpy(R_y, R_x, sizeof(R_x));
    memcpy(min_var_y, min_var_x, sizeof(min_var_x));

    /* 初始化X轴卡尔曼滤波器：2维状态[位置,速度], 1维控制[加速度], 1维观测[速度] */
    Kalman_Filter_Init(&KF_PosX, 2, 1, 1);
    KF_PosX.UseAutoAdjustment = 0;
    Init_KF_Matrices(&KF_PosX, F_x, B_x, Q_x, P_x, H_x, R_x, min_var_x, 2, 1);

    /* 初始化Y轴卡尔曼滤波器 */
    Kalman_Filter_Init(&KF_PosY, 2, 1, 1);
    KF_PosY.UseAutoAdjustment = 0;
    Init_KF_Matrices(&KF_PosY, F_y, B_y, Q_y, P_y, H_y, R_y, min_var_y, 2, 1);

    Odometer.initialized = 1;
}

void Chassis_Inverse_Kinematics(float motor_speed[4],
                                 float *vx, float *vy, float *vw)
{
    /*
     * 根据Chassis_SetSpeed()的正解公式反推：
     *
     * 正解（底盘速度→电机转速）：
     *   Vp = Vw * (L + W)
     *   M0 = (-Vx - Vy*(1+G) - Vp) / (π*R) * 60 * dec
     *   M1 = ( Vx - Vy*(1+G) - Vp) / (π*R) * 60 * dec
     *   M2 = (-Vx + Vy*(1-G) - Vp) / (π*R) * 60 * dec
     *   M3 = ( Vx + Vy*(1-G) - Vp) / (π*R) * 60 * dec
     *
     * 逆解（电机转速→底盘速度，忽略G_COMPENSATION）：
     *   Vx = (-M0 + M1 - M2 + M3) / 4
     *   Vy = (-M0 - M1 + M2 + M3) / 4
     *   Vw = (-M0 - M1 - M2 - M3) / (4 * (L+W))
     */

    /* 转换为线速度 (m/s) */
    float v[4];
    for (int i = 0; i < 4; i++) {
        v[i] = motor_speed[i] * RPM_TO_MS;
    }

    float L_plus_W = CHASSIS_LENGTH + CHASSIS_WIDTH;

    /* 逆解 */
    *vx = (-v[0] + v[1] - v[2] + v[3]) / 4.0f;
    *vy = (-v[0] - v[1] + v[2] + v[3]) / 4.0f;
    *vw = (-v[0] - v[1] - v[2] - v[3]) / (4.0f * L_plus_W);
}

void Odometer_Update(void)
{
    if (!Odometer.initialized)
        return;

    /*------------------------------------------------------
     * 步骤1：获取电机速度并解算底盘速度（逆运动学）
     *------------------------------------------------------*/
    float motor_speeds[4];
    for (int i = 0; i < 4; i++) {
        motor_speeds[i] = (float)chassis.ChassisMotors[i].speed;
    }

    float vx_body, vy_body, vw;
    Chassis_Inverse_Kinematics(motor_speeds, &vx_body, &vy_body, &vw);

    Odometer.vel_body_x = vx_body;
    Odometer.vel_body_y = vy_body;
    Odometer.vel_w = vw;

    /*------------------------------------------------------
     * 步骤2：坐标转换（机体坐标系→导航坐标系）
     *------------------------------------------------------*/
    float yaw_rad = INS.Yaw * 3.14159265f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    /* 旋转矩阵：[nav] = R(yaw) * [body] */
    float vx_nav = vx_body * cos_yaw - vy_body * sin_yaw;
    float vy_nav = vx_body * sin_yaw + vy_body * cos_yaw;

    /*------------------------------------------------------
     * 步骤3：X轴卡尔曼滤波
     *   状态: [位置, 速度]
     *   控制输入: IMU X轴加速度
     *   观测量: 编码器解算的X轴速度
     *------------------------------------------------------*/
    KF_PosX.ControlVector[0] = INS.MotionAccel_n[X];
    KF_PosX.MeasuredVector[0] = vx_nav;
    Kalman_Filter_Update(&KF_PosX);

    Odometer.pos_x = KF_PosX.FilteredValue[0];
    Odometer.vel_x = KF_PosX.FilteredValue[1];

    /*------------------------------------------------------
     * 步骤4：Y轴卡尔曼滤波
     *------------------------------------------------------*/
    KF_PosY.ControlVector[0] = INS.MotionAccel_n[Y];
    KF_PosY.MeasuredVector[0] = vy_nav;
    Kalman_Filter_Update(&KF_PosY);

    Odometer.pos_y = KF_PosY.FilteredValue[0];
    Odometer.vel_y = KF_PosY.FilteredValue[1];

    /*------------------------------------------------------
     * 步骤5：更新航向角（直接使用INS解算结果）
     *------------------------------------------------------*/
    Odometer.yaw = INS.Yaw;
    Odometer.yaw_total = INS.YawTotalAngle;
}

void Odometer_Reset(void)
{
    memset(&Odometer, 0, sizeof(Odometer_t));

    /* 重置滤波器状态 */
    memset(KF_PosX.xhat_data, 0, sizeof(float) * 2);
    memset(KF_PosY.xhat_data, 0, sizeof(float) * 2);

    /* 重置协方差矩阵 */
    float P_init[4] = {1.0f, 0.0f, 0.0f, 1.0f};
    memcpy(KF_PosX.P_data, P_init, sizeof(P_init));
    memcpy(KF_PosY.P_data, P_init, sizeof(P_init));

    Odometer.initialized = 1;
}

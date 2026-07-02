# ROBOCON_R2

ABU ROBOCON 亚太机器人竞赛 R2 号机器人固件，基于 STM32F407，使用 STM32CubeMX 生成 HAL 初始化代码，CMake + GCC ARM 工具链构建。

任务调度基于 TIM7 定时器中断（1ms 基准），通过计数器分频实现多周期任务。

## 硬件平台

| 项目 | 规格 |
|---|---|
| MCU | STM32F407IGH6TR (Cortex-M4F, 168 MHz) |
| 调试 | ST-Link SWD (OpenOCD) |
| 通信总线 | CAN1 @ 1 Mbaud (DJI 电调) |
| 遥控接收 | FS-i6X (SBUS @ 100000 baud, USART3) |
| IMU | BMI088 (SPI1, 加速度计 6G / 陀螺仪 2000dps, 含恒温加热) |
| 上位机通信 | USB OTG FS (USB CDC) |
| 调试串口 | Vofa+ (USART1, JustFloat 协议) |

## 机器人功能模块

| 模块 | 说明 |
|---|---|
| 麦克纳姆轮底盘 | 4 × DJI M3508 (ID 1-4), 全向移动, IMU 航向锁定 PID |
| 车身升降 | 前后气缸, GPIO 控制电磁阀 (PE9/PE11) |
| 3-DOF 吸盘平台 | 3 × M3508 (ID 6-8) + 电磁阀 (PE13, 通电真空/断电释放) |
| 升降台 | 1 × M3508 (ID 5), 高度 mm 级控制 (14mm/转) |
| 气动夹爪 | 电磁阀夹取 (PE14) + 舵机旋转手腕 (TIM8 CH1, ±135°) |

## 项目结构

```
ROBOCON_R2/
├── Core/                      # STM32CubeMX 生成的 HAL 初始化
│   ├── Inc/
│   └── Src/
├── Drivers/                   # STM32 HAL / CMSIS 库
├── Middlewares/               # USB 中间件
├── USB_DEVICE/                # USB CDC 设备类
├── User/                      # 用户应用代码
│   ├── APP/                   # 应用层
│   │   ├── app_task.c/h       # 任务调度 (remote_control / pc_control 切换)
│   │   ├── Chassis.c/h        # 麦轮底盘运动学 + 航向 PID
│   │   ├── ins_task.c/h       # 惯性导航 (四元数 EKF, IMU 恒温控制)
│   │   ├── task_it.c/h        # TIM7 中断任务分发 (1ms 基准, 多周期调度)
│   │   ├── car_lifting.c/h    # 车身升降控制
│   │   └── debug.c/h          # 调试测试框架 (已弃用)
│   ├── Device/                # 设备驱动层
│   │   ├── dev_dji_motor/     # DJI M3508/M2006 CAN 驱动 (dvc_dji_motor.h/c)
│   │   ├── dev_imu/           # BMI088 IMU 驱动 (BMI088driver.h/c)
│   │   ├── dev_remote/        # SBUS 遥控器解析 (dvc_remote.h/c)
│   │   ├── dev_pc/            # USB 上位机协议 (dev_pc.h/c)
│   │   ├── dev_3DOF_SC/       # 三轴吸盘平台
│   │   ├── dev_air_finger/    # 气动夹爪 + 舵机
│   │   ├── dev_lift_table/    # 电机升降台
│   │   └── dev_vofa/          # Vofa+ 调试工具
│   └── Middleware/            # 中间件
│       ├── Algorithm/         # PID / 模糊PID / 阻抗控制 / EKF / 前馈 / LDOB
│       └── Driver/            # CAN / UART / SPI / PWM 底层驱动
├── CMakeLists.txt             # CMake 构建配置
├── CMakePresets.json           # CMake 预设
├── ROBOCON_R2.ioc             # STM32CubeMX 工程文件
├── STM32F407XX_FLASH.ld       # GCC 链接脚本
├── openocd.cfg                # OpenOCD 调试配置
└── startup_stm32f407xx.s      # 启动汇编
```

## 构建

### 依赖

- [ARM GCC 工具链](https://developer.arm.com/downloads/-/gnu-rm) (`arm-none-eabi-gcc`)
- [CMake](https://cmake.org/) ≥ 3.22
- [OpenOCD](http://openocd.org/) (调试/烧录)

### 编译

```bash
# 配置 (使用 CMakePresets.json 中的预设)
cmake --preset Debug

# 构建
cmake --build build/Debug
```

### 烧录 & 调试

```bash
# 烧录
openocd -f openocd.cfg -c "program build/Debug/ROBOCON_R2.elf verify reset exit"

# 启动 GDB 调试服务器
openocd -f openocd.cfg
```

## 控制模式

固件支持两种编译时控制模式 (`app_task.h`):

| 宏 | 值 | 说明 |
|---|---|---|
| `remote_control` | 1 (当前启用) | FS-i6X 遥控器主控, 通过 SBUS 接收 |
| `pc_control` | 0 (当前禁用) | 上位机 USB CDC 指令控制 |

通过修改 `app_task.h` 中的宏值 (0/1) 切换控制源。

## 任务调度

任务调度基于 TIM7 定时器中断（1ms 周期），在 `Task_it_callback()` 中通过计数器分频实现多周期任务：

| 周期 | 任务 |
|---|---|
| 2ms | DJI 电机 CAN 指令发送 (`DJ_MotorRun`) |
| 5ms | 航向 PID 计算 + 底盘电机指令 (`Chassis_YawControl` + `Chassis_Run`) |
| 1s | 运行指示灯心跳 |

惯性导航 (`INS_Task`) 在 `App_Task_Run` 中以 2ms DWT 定时独立运行，IMU 恒温控制以 500Hz 执行。

## 外设映射

| 外设 | 引脚 | 用途 |
|---|---|---|
| CAN1 | PD0/PD1 | DJI 电调总线 |
| USART1 | PA9/PB7 | Vofa 调试串口 |
| USART3 | PC10/PC11 | SBUS 遥控接收 |
| SPI1 | PB3/PB4/PA7 | BMI088 IMU |
| TIM8 CH1 | PC6 | 舵机 PWM (50 Hz) |
| TIM10 CH1 | PF6 | IMU 加热器 PWM (1 kHz) |
| USB OTG FS | PA11/PA12 | 上位机通信 |
| PE9/PE11 | - | 气缸电磁阀 (升降) |
| PE13 | - | 吸盘电磁阀 (通电真空/断电释放) |
| PE14 | - | 夹爪电磁阀 |
| PC8 | - | 激光 |

## 算法库

`User/Middleware/Algorithm/controller.h/c` 提供以下控制器：

- **PID** — 基础 PID（支持积分限幅、梯形积分、微分先行等可选特性）
- **TDPID** — 跟踪微分器 PID（跟踪微分器 + PID 组合）
- **FastPID** — 快速增量式 PID
- **阻抗控制** — 二阶阻抗模型
- **模糊 PID** — 7×7 规则表，可挂载到 Ctrl_PID_t
- **前馈控制** — 一阶 + 二阶前馈
- **LDOB** — 线性扰动观测器
- **跟踪微分器** — TD 过渡过程

## License

MIT

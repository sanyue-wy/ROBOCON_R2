cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#



# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME ROBOCON_R2)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/Middleware/Driver/drv_can.c
        User/Middleware/Driver/drv_tim.c
        User/Middleware/Driver/drv_usart.c
        User/Middleware/Driver/FastMathFunctions.c
        User/Middleware/Algorithm/alg_pid.c
        User/Device/dev_vofa/dev_vofa.c
        User/APP/debug.h
        User/APP/debug.c
        User/Device/dev_dji_motor/dvc_dji_motor.c
        User/Device/dev_dji_motor/dvc_dji_motor_date.h
        User/APP/Chassis.c
        User/APP/task_it.h
        User/APP/task_it.c
        User/Middleware/Driver/arm_math.h
        User/Middleware/Driver/user_lib.c
        User/Middleware/Driver/transfer_function.c
        User/Middleware/Driver/BMI088reg.h
        User/Middleware/Driver/BMI088Middleware.c
        User/Middleware/Algorithm/QuaternionEKF.h
        User/Middleware/Algorithm/controller.h
        User/Device/dev_imu/BMI088driver.c
        User/Middleware/Driver/bsp_dwt.c
        User/Middleware/Algorithm/controller.c
        User/Middleware/Algorithm/kalman_filter.c
        User/Middleware/Algorithm/QuaternionEKF.c
        User/APP/ins_task.c
        User/Middleware/Driver/bsp_PWM.c
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        User/APP
        User/Device
        User/Middleware/Algorithm
        User/Middleware/Driver
        User/Device/dev_dji_motor
        User/Device/dev_vofa
        User/Device/dev_imu
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# 全局宏定义：STM32F407 + CMSIS-DSP 必需
add_definitions(
        -DSTM32F407xx
        -DUSE_HAL_DRIVER_STM32F407
        -DARM_MATH_CM4        # F407 内核：Cortex-M4

)

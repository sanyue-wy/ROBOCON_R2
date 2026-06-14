//
// Created by wy on 2026/6/11.
//

#ifndef dev_air_finger_H
#define dev_air_finger_H
#include "gpio.h"
#include "tim.h"

void air_finger_init(void);
void air_finger_grip(void);
void air_finger_release(void);


void Servo_Init(void);
void Servo_SetAngle_135(uint32_t channel, int16_t angle);
void Servo_SetPulse(uint32_t channel, uint16_t us);


#endif //dev_air_finger_H
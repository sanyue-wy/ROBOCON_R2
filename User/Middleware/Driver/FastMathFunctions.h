/**
 * 1.不要问为什么
 * 2.问就是为了提高单片机运算速率
 * 3.不可随意更改
 */

#ifndef __FAST_MATH_FUNCTIONS_H
#define __FAST_MATH_FUNCTIONS_H

#include "stdint.h"
#include "stdbool.h"
#include "math.h"

#define FAST_MATH_TABLE_SIZE 512

#define PI 3.14159265358979f
#define PI_HALF 1.5707963267948966192313f

// 快速正弦
float fast_sin(float radians);
// 快速余弦
float fast_cos(float radians);
// 快速平方根，不如sqrtf
bool fast_sqrt(float in, float *pOut);
// 快速反正切, 比atan快不了多少
float fast_atan(float x);
// 快速反正切2
bool fast_atan2(float y, float x, float *result);

#endif

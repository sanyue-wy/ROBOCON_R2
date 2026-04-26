//
// Created by wy on 2026/4/25.
//

#ifndef debug_H
#define debug_H
#include "dev_vofa.h"
#include  "drv_tim.h"
#include  "drv_can.h"
#include  "drv_usart.h"

//测试模式
#define vofa_debug 1

void debug_init(void);
void debug_run(void);



#endif //debug_H
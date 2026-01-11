//
// Created by linglitel on 2026/1/10.
//

#ifndef SMARTCAR_APP_PID_H
#define SMARTCAR_APP_PID_H

#include "math.h"

#define LIMIT(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float error_prev; // 上一次误差 (用于D)
    float integral; // 积分累加 (用于I)

    float output_limit; // 输出限幅 (例如 PWM 最大 4250)
    float integral_limit; // 积分限幅 (防积分饱和)
} PID_Config_t;

float PID_Calculate(PID_Config_t *pid, float target, float current);

#endif //SMARTCAR_APP_PID_H

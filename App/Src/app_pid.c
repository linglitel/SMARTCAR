//
// Created by linglitel on 2026/1/10.
//

#include "app_pid.h"

float PID_Calculate(PID_Config_t *pid, float target, float current) {
    float error = target - current;
    // 1. 积分 (I)
    pid->integral += error;

    // 积分限幅
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    // 3. 微分 (D)
    float derivative = error - pid->error_prev;
    pid->error_prev = error;

    // 4. 计算输出
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 5. 输出限幅
    if (output > pid->output_limit) output = pid->output_limit;
    else if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

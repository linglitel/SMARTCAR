//
// Created by linglitel on 2026/1/10.
//

#include "app_pid.h"

float PID_Calculate(PID_Config_t *pid, float target, float current) {
    float error = target - current;

    // 1. 死区处理 (位置环常用，防止在终点抖动)
    if (fabsf(error) < pid->dead_zone) {
        error = 0.0f;
        // 进入死区时，建议清零积分或保持，这里简单处理
        pid->integral = 0.0f;
        return 0.0f;
    }

    // 2. 积分 (I)
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

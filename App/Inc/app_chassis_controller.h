//
// Created by linglitel on 2026/1/11.
//

#ifndef SMARTCAR_APP_CHASSIS_CONTROLLER_H
#define SMARTCAR_APP_CHASSIS_CONTROLLER_H

#include "app_pid.h"
#include "stdint.h"
#include "app_motor_controller.h"

typedef struct {
    // --- 控制目标 ---
    float target_heading; // 目标朝向 (度)
    float current_heading; // 当前朝向 (度)

    // --- PID ---
    PID_Config_t pid_heading; // 转向环 PID

    // --- 状态 ---
    uint8_t is_turning; // 是否正在执行转向任务
    float tolerance; // 角度容忍度 (比如 1.0度)
} Chassis_Controller_t;

void Chassis_Controller_Move_Turn(float angle, float max_speed);
void Chassis_Heading_Control_ISR(void);

#endif //SMARTCAR_APP_CHASSIS_CONTROLLER_H

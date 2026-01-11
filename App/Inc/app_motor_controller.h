//
// Created by linglitel on 2026/1/10.
//

#ifndef SMARTCAR_APP_MOTOR_CONTROLLER_H
#define SMARTCAR_APP_MOTOR_CONTROLLER_H

#include "main.h"
#include "app_pid.h"

// --- 枚举：控制模式 ---
typedef enum {
    CTRL_MODE_STOP = 0, // 停机 (PWM=0)
    CTRL_MODE_SPEED, // 纯速度模式
    CTRL_MODE_POSITION // 位置模式
} Control_Mode_t;

typedef enum {
    MOTOR_STATE_IDLE = 0, // 空闲 (到位了/停机)
    MOTOR_STATE_BUSY = 1 // 忙碌 (正在跑)
} Motor_State_t;

// --- 结构体：电机完整控制器 ---
typedef struct {
    // 1. 硬件资源
    TIM_HandleTypeDef *htim; // PWM 定时器 (TIM1/8)
    uint32_t channel_1; // PWM 通道1 (TIM_CHANNEL_1...)
    uint32_t channel_2; // PWM 通道2 (TIM_CHANNEL_1...)
    TIM_HandleTypeDef *htim_enc; // 编码器定时器 (TIM2/3/4/5)

    // 2. 状态数据
    int32_t total_count; // 总累计脉冲数 (位置)
    int16_t current_speed; // 当前速度 (脉冲/ms)
    uint16_t last_enc_cnt; // 上一次读取的编码器寄存器值

    // 3. 控制目标
    Control_Mode_t mode; // 当前模式
    int32_t target_speed; // 目标速度 (Speed模式直接用，Pos模式由外环生成)
    int32_t target_position; // 目标位置 (仅Pos模式用)
    int32_t max_speed_in_pos_mode; // 位置模式下的最大巡航速度

    // 4. PID 实例
    PID_Config_t pid_speed; // 速度环 (内环)
    PID_Config_t pid_pos; // 位置环 (外环)

    // 5. 最终输出
    float pwm_out; // 计算出的 PWM (-4250 ~ +4250)

    // --- 2. 新增：状态管理成员 ---
    volatile Motor_State_t state; // 当前状态 (volatile很重要，因为中断会改它)
    float tolerance; // 到位容忍度 (脉冲数)，比如 50
    uint32_t arrival_timer; // (可选) 用于计时防抖，确保停稳了才算到位
} Motor_Controller_t;

void Motor_Controller_Init(Motor_Controller_t *motor, TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2,
                           TIM_HandleTypeDef *htim_enc, float s_kp, float s_ki, float s_kd, float s_ol, float s_il,
                           float p_kp, float p_ki, float p_kd, float p_ol, float tolerance);

void Motor_Controller_ISR(Motor_Controller_t *motor);

void Motor_Controller_Move_Forward(Motor_Controller_t *motor, Control_Mode_t mode, int32_t target_position,
                                   int32_t target_speed);

void Motor_Controller_Stop();
#endif //SMARTCAR_APP_MOTOR_CONTROLLER_H

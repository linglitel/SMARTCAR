//
// Created by linglitel on 2026/1/10.
//

#include "app_motor_controller.h"

#include "bsp_motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim20;

void Motor_Controller_Init(Motor_Controller_t *motor, TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2,
                           TIM_HandleTypeDef *htim_enc, float s_kp, float s_ki, float s_kd, float s_ol, float s_il,
                           float p_kp, float p_ki, float p_kd, float p_ol, float tolerance) {
    motor->htim = htim;
    motor->channel_1 = channel1;
    motor->channel_2 = channel2;
    motor->htim_enc = htim_enc; //todo 拆分编码器优化结构
    motor->total_count = 0;
    motor->current_speed = 0;
    motor->last_enc_cnt = 0;
    motor->target_speed = 0;
    motor->target_position = 0;
    motor->max_speed_in_pos_mode = 0.0f;
    motor->mode = CTRL_MODE_STOP;
    motor->pwm_out = 0.0f;
    motor->pid_speed.Kp = s_kp; // 举例：速度差1个脉冲，PWM加10
    motor->pid_speed.Ki = s_ki; // 消除静差
    motor->pid_speed.Kd = s_kd; // 有刷电机通常不需要D，或者给很小
    motor->pid_speed.output_limit = s_ol; // PWM 最大值
    motor->pid_speed.integral_limit = s_il;

    // 位置环 (外环) - 只用 P 通常就够了，或者弱 PI
    motor->pid_pos.Kp = p_kp; // 举例：距离差100个脉冲，给速度环下发 50 的速度
    motor->pid_pos.Ki = p_ki; // 位置环通常不用 I，除非要抵抗重力
    motor->pid_pos.Kd = p_kd;
    motor->pid_pos.output_limit = p_ol; // 这个limit其实限制的是“最大理论请求速度”
    motor->tolerance = tolerance; // 允许 5个脉冲的定位误差
}

void Motor_Controller_ISR(Motor_Controller_t *motor) { //todo 移动到chassis
    // --- Step 1: 读取编码器反馈 (速度 & 位置) ---
    uint16_t enc_now = __HAL_TIM_GET_COUNTER(motor->htim_enc);

    // 关键：处理溢出！使用 int16_t 强制转换计算差值
    // 无论 16位(TIM3/4) 还是 32位(TIM2/5) 定时器，
    // 只要转速在 1ms 内不超过 32767 个脉冲，这个差值就是对的。
    int16_t delta = (int16_t) (enc_now - motor->last_enc_cnt);

    motor->last_enc_cnt = enc_now;
    motor->current_speed = delta; // 速度单位：Counts per 1ms
    motor->total_count += delta; // 位置单位：Total Counts


    // --- Step 2: 运行 PID ---
    int32_t speed_ref = 0.0f; // 最终给速度环的参考值

    switch (motor->mode) {
        case CTRL_MODE_STOP:
            motor->pwm_out = 0.0f;
            // 停机时重置积分，防止再次启动时冲出去
            motor->pid_speed.integral = 0.0f;
            motor->pid_pos.integral = 0.0f;
            break;

        case CTRL_MODE_SPEED:
            // 纯速度模式：直接使用设定的目标速度
            motor->state = MOTOR_STATE_BUSY;
            speed_ref = motor->target_speed;

            // 运行内环 (速度环)
            motor->pwm_out = PID_Calculate(&motor->pid_speed, (float) speed_ref, motor->current_speed);
            break;

        case CTRL_MODE_POSITION:
            // 位置模式 (串级 PID)

            float pos_error = fabsl(motor->target_position - motor->total_count);
            if (pos_error <= motor->tolerance) {
                motor->state = MOTOR_STATE_IDLE;
                speed_ref = 0.0f;
                motor->pwm_out = 0.0f;

                // 【关键】到达目标后，清除速度环积分，防止由于长时间积分导致的电机异响或抖动
                motor->pid_speed.integral = 0.0f;
                motor->pid_pos.integral = 0.0f;
            } else {
                motor->state = MOTOR_STATE_BUSY;
                // A. 外环 (位置环): 输入位置误差，输出期望速度
                float desired_speed =
                        PID_Calculate(&motor->pid_pos, motor->target_position, (float) motor->total_count);

                // B. 限制最大巡航速度 (这实现了"指定速度跑距离")
                // 比如 PID 算出要跑 1000的速度，但你限制了 500，它就乖乖跑 500
                speed_ref = LIMIT(desired_speed, -motor->max_speed_in_pos_mode, motor->max_speed_in_pos_mode);

                // C. 内环 (速度环): 输入期望速度，输出 PWM
                motor->pwm_out = PID_Calculate(&motor->pid_speed, speed_ref, (float) motor->current_speed);
                break;
            }
    }
    Motor_Set_PWM(motor->htim, motor->channel_1, motor->channel_2, motor->pwm_out);
}

void Motor_Controller_Move_Forward(Motor_Controller_t *motor, Control_Mode_t mode, int32_t target_position,
                                   int32_t target_speed) {
    motor->mode = mode;
    motor->state = MOTOR_STATE_BUSY;
    motor->target_position = target_position + motor->total_count;
    motor->pid_pos.error_prev = 0.0f;
    motor->target_speed = target_speed;
    motor->max_speed_in_pos_mode = target_speed;
}

void Motor_Controller_Stop() {
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_4);
}

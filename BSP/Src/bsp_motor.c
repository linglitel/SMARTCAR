//
// Created by linglitel on 2026/1/10.
//

#include "bsp_motor.h"

void bsp_motor_init(TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2, TIM_HandleTypeDef *htim_enc) {

}

void Motor_Set_PWM(TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2, float pwm_val) {
    uint32_t pwm_max = __HAL_TIM_GET_AUTORELOAD(htim);
    if (pwm_val >= 0) {
        __HAL_TIM_SET_COMPARE(htim, channel1, (uint32_t)pwm_max);
        __HAL_TIM_SET_COMPARE(htim, channel2, (uint32_t)pwm_val);
    }else {
        __HAL_TIM_SET_COMPARE(htim, channel1, (uint32_t)pwm_val);
        __HAL_TIM_SET_COMPARE(htim, channel2, (uint32_t)pwm_max);
    }
}

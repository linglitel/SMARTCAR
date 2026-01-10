//
// Created by linglitel on 2026/1/10.
//

#ifndef SMARTCAR_BSP_MOTOR_H
#define SMARTCAR_BSP_MOTOR_H

#include "app_motor_controller.h"
#include "main.h"



void Motor_Set_PWM(TIM_HandleTypeDef *htim, uint32_t channel1, uint32_t channel2, float pwm_val);

#endif //SMARTCAR_BSP_MOTOR_H

//
// Created by linglitel on 2026/1/11.
//

#ifndef SMARTCAR_APP_UTILS_H
#define SMARTCAR_APP_UTILS_H

#include "stdint.h"
#include "app_motor_controller.h"

uint8_t Motor_Is_Idle(void);

float Math_Normalize_360(float angle);

float Math_Calc_Shortest_Error(float target, float current);

#endif //SMARTCAR_APP_UTILS_H

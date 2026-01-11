//
// Created by linglitel on 2026/1/11.
//

#include "app_utils.h"

extern Motor_Controller_t Motor_Controller_1;
extern Motor_Controller_t Motor_Controller_2;
extern Motor_Controller_t Motor_Controller_3;
extern Motor_Controller_t Motor_Controller_4;

uint8_t Motor_Is_Idle(void) {
    if (Motor_Controller_1.state == MOTOR_STATE_IDLE &&
        Motor_Controller_2.state == MOTOR_STATE_IDLE &&
        Motor_Controller_3.state == MOTOR_STATE_IDLE &&
        Motor_Controller_4.state == MOTOR_STATE_IDLE) {
        return 1;
    }
    return 0;
}

float Math_Normalize_360(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

float Math_Calc_Shortest_Error(float target, float current) {
    float err = target - current;
    if (err > 180.0f) err -= 360.0f;
    else if (err < -180.0f) err += 360.0f;
    return err;
}

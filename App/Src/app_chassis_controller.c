//
// Created by linglitel on 2026/1/11.
//

#include "app_chassis_controller.h"

#include "app_utils.h"
#include "bsp_imu.h"

Chassis_Controller_t chassis;

extern Motor_Controller_t Motor_Controller_1;
extern Motor_Controller_t Motor_Controller_2;
extern Motor_Controller_t Motor_Controller_3;
extern Motor_Controller_t Motor_Controller_4;
extern IMU_Data_t IMU_Data;

void Chassis_Controller_Move_Turn(float angle, float max_speed) {
    chassis.current_heading = IMU_Data.yaw;
    chassis.target_heading = Math_Normalize_360(chassis.current_heading + angle);
    chassis.pid_heading.Kp = 15.0f; // 误差1度，输出15的速度
    chassis.pid_heading.Ki = 0.05f; // 消除稳态误差
    chassis.pid_heading.Kd = 2.0f; // 抑制超调
    chassis.pid_heading.output_limit = max_speed;
    chassis.pid_heading.integral_limit = max_speed / 2.0f;
    chassis.tolerance = 0.5f; // 0.5度死区

    chassis.pid_heading.integral = 0.0f;
    chassis.pid_heading.error_prev = 0.0f;
    Motor_Controller_1.mode = CTRL_MODE_SPEED;
    Motor_Controller_1.state = MOTOR_STATE_BUSY;
    Motor_Controller_2.mode = CTRL_MODE_SPEED;
    Motor_Controller_2.state = MOTOR_STATE_BUSY;
    Motor_Controller_3.mode = CTRL_MODE_SPEED;
    Motor_Controller_3.state = MOTOR_STATE_BUSY;
    Motor_Controller_4.mode = CTRL_MODE_SPEED;
    Motor_Controller_4.state = MOTOR_STATE_BUSY;
}

void Chassis_Heading_Control_ISR(void) {
    if (chassis.is_turning == 0) return;
    chassis.current_heading = IMU_Data.yaw;
    // 1. 计算最短路径误差 (-180 ~ +180)
    // 正数表示：目标在右边 (需要右转)
    // 负数表示：目标在左边 (需要左转)
    float err = Math_Calc_Shortest_Error(chassis.target_heading, chassis.current_heading);
    if (fabsf(err) <= chassis.tolerance) {
        chassis.is_turning = 0; // 任务结束

        // 停车
        Motor_Controller_1.target_speed = 0;
        Motor_Controller_1.state = MOTOR_STATE_IDLE;
        Motor_Controller_2.target_speed = 0;
        Motor_Controller_2.state = MOTOR_STATE_IDLE;
        Motor_Controller_3.target_speed = 0;
        Motor_Controller_3.state = MOTOR_STATE_IDLE;
        Motor_Controller_4.target_speed = 0;
        Motor_Controller_4.state = MOTOR_STATE_IDLE;
        return;
    }
    float out = PID_Calculate(&chassis.pid_heading, err, 0.0f);
    Motor_Controller_1.target_speed = +out;
    Motor_Controller_2.target_speed = -out;
    Motor_Controller_3.target_speed = +out;
    Motor_Controller_4.target_speed = -out;
}

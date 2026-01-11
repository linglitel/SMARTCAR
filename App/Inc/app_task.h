//
// Created by linglitel on 2026/1/11.
//

#ifndef SMARTCAR_APP_TASK_H
#define SMARTCAR_APP_TASK_H

#include "FreeRTOS.h"
#include "queue.h"
#include "freertos_os2.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include "bsp_imu.h"
#include "app_utils.h"

typedef enum {
    CMD_FORWARD = 1, // 直行
    CMD_TURN = 2, // 转向
    CMD_WAIT = 3, // 原地等待 (比如停顿2秒)
    CMD_CLEAR_ALL = 99 // 【重要】急停并清空所有待办任务
} CmdType_t;

typedef struct {
    CmdType_t type;
    int32_t param1; // 模式
    int32_t param2; // 距离(m) 或 角度(deg) 或 等待时间(ms)
    int32_t param3; // 速度(m/s) 或 预留
} Command_Packet_t;

extern QueueHandle_t xCmdQueue;

void Task_CmdExecutor(void *argument);

void App_Task_IMU(void);

#endif //SMARTCAR_APP_TASK_H

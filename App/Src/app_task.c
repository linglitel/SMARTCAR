//
// Created by linglitel on 2026/1/11.
//

#include "app_task.h"

extern Motor_Controller_t Motor_Controller_1;
extern Motor_Controller_t Motor_Controller_2;
extern Motor_Controller_t Motor_Controller_3;
extern Motor_Controller_t Motor_Controller_4;

QueueHandle_t xCmdQueue;

void Task_CmdExecutor(void *argument) {
    Command_Packet_t current_cmd;

    while (1) {
        // --- 1. 获取指令 ---
        // 如果队列是空的，任务会挂起 (Blocked)，不消耗 CPU
        // 一旦上位机发来数据，这里会立刻苏醒
        if (xQueueReceive(xCmdQueue, &current_cmd, portMAX_DELAY) == pdTRUE) {
            // --- 2. 派发任务 ---
            switch (current_cmd.type) {
                case CMD_FORWARD:
                    // 调用你之前写的相对移动函数
                    Motor_Controller_Move_Forward(&Motor_Controller_1, current_cmd.param1, current_cmd.param2,
                                                  current_cmd.param3);
                    Motor_Controller_Move_Forward(&Motor_Controller_2, current_cmd.param1, current_cmd.param2,
                                                  current_cmd.param3);
                    Motor_Controller_Move_Forward(&Motor_Controller_3, current_cmd.param1, current_cmd.param2,
                                                  current_cmd.param3);
                    Motor_Controller_Move_Forward(&Motor_Controller_4, current_cmd.param1, current_cmd.param2,
                                                  current_cmd.param3);
                    break;

                case CMD_TURN:
                    break;

                case CMD_WAIT:
                    // 纯等待，什么都不做
                    //todo 需要添加逻辑
                    break;
                case CMD_CLEAR_ALL:
                    Motor_Controller_Stop();
                    xQueueReset(xCmdQueue);
                    break;
            }

            // --- 3. 阻塞等待：直到物理动作执行完毕 ---
            // 这就是实现“按顺序执行”的关键！

            if (current_cmd.type == CMD_WAIT) {
                // 如果是等待指令，直接延时
                vTaskDelay(pdMS_TO_TICKS((uint32_t)current_cmd.param1));
            } else {
                // 如果是运动指令，轮询电机状态
                while (Motor_Is_Idle() == 0) // 只要车还在动
                {
                    // 检查队列里是否有“急停”插队？(可选的高级功能)
                    // if (Check_Emergency()) break;

                    vTaskDelay(10); // 释放 CPU 给 PID 中断和其他任务
                }
            }

            // --- 4. 动作完成，循环回去取下一个 ---
            // 可以在这里通过串口给上位机回一个 "CMD_ID_xxx DONE"
            printf("Cmd Done\n");
        }
    }
}

void App_Init_FreeRTOS(void) {
    // 1. 创建队列
    // 深度设为 50，表示最多能一次性缓存 50 个动作，足够跑完一个比赛流程了
    xCmdQueue = xQueueCreate(50, sizeof(Command_Packet_t));

    // 2. 创建任务
    xTaskCreate(Task_CmdExecutor, "Executor", 256, NULL, osPriorityNormal, NULL);
    // 假设你还有个 Task_Rx 负责收串口
}

void App_Task_IMU(void) {
    for (;;) {
        osDelay(5);
    }
}

//
// Created by linglitel on 2026/1/10.
//

#ifndef SMARTCAR_BSP_TEMP_H
#define SMARTCAR_BSP_TEMP_H

#include "main.h"

typedef struct {
    // --- 硬件资源 ---
    volatile uint16_t *dma_ptr; // 指向 ADC1 buffer 的第5个元素

    // --- 物理参数 (STM32G4出厂校准值) ---
    // 这些地址在 G4 数据手册中定义，初始化时读取即可
    int32_t ts_cal1; // 30度时的ADC值
    int32_t ts_cal2; // 110度或130度时的ADC值 (具体看型号，G4通常是110或130)
    float slope; // 温度/ADC值 的斜率

    // --- 输出数据 ---
    float temperature_C; // 摄氏度
} Chip_Temp_t;

void Chip_Temp_Init(Chip_Temp_t *temp, volatile uint16_t *data_ptr);

void Chip_Temp_Enable(Chip_Temp_t *temp);

void Chip_Temp_Update(Chip_Temp_t *temp);

void Chip_Temp_DeInit(Chip_Temp_t *temp);

#endif //SMARTCAR_BSP_TEMP_H

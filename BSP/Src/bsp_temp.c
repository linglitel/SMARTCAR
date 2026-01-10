//
// Created by linglitel on 2026/1/10.
//

#include "bsp_temp.h"

void Chip_Temp_Init(Chip_Temp_t *temp, volatile uint16_t *data_ptr) {
    temp->dma_ptr = data_ptr;

    // 读取芯片出厂固化的校准值
    temp->ts_cal1 = (int32_t) (*TEMPSENSOR_CAL1_ADDR);
    temp->ts_cal2 = (int32_t) (*TEMPSENSOR_CAL2_ADDR);

    // 预计算斜率，减少运行时运算
    // Slope = (T2 - T1) / (ADC2 - ADC1)
    temp->slope = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) /
                  (float) (temp->ts_cal2 - temp->ts_cal1);
}

/*
 * 初始化即启用,保持函数风格
 */
void Chip_Temp_Enable(Chip_Temp_t *temp) {
}

void Chip_Temp_Update(Chip_Temp_t *temp) {
    uint16_t raw_adc = *(temp->dma_ptr);

    // 线性插值计算: T = 30 + (Raw - Cal1) * Slope
    // 注意：G4的温度传感器供电必须稳定，建议ADC参考电压准确
    float calc_temp = TEMPSENSOR_CAL1_TEMP + ((float) ((int32_t) raw_adc - temp->ts_cal1) * temp->slope);

    // 简单滤波 (温度变化极慢，系数可以很小)
    temp->temperature_C = (0.01f * calc_temp) + (0.99f * temp->temperature_C);
}

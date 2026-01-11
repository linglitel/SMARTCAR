//
// Created by linglitel on 2026/1/11.
//

#ifndef SMARTCAR_BSP_IMU_H
#define SMARTCAR_BSP_IMU_H

#include "stdint.h"
#include "math.h"

#define IMU_DT_SEC          0.001f
#define FUSION_ALPHA        0.98f

typedef struct {
    float roll; /* rad */
    float pitch; /* rad */
    float yaw; /* rad*/
} IMU_Data_t;
#endif //SMARTCAR_BSP_IMU_H

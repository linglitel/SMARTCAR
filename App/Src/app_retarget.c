//
// Created by linglitel on 2026/1/10.
//

#include "app_retarget.h"
#include "main.h"
#include "string.h"

extern UART_HandleTypeDef huart2;

int _write(int fd, char *ptr, int len) {
    (void) fd;
    HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    return len;
}

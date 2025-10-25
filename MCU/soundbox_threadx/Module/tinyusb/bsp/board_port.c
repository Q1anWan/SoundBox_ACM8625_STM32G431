#include "board_api.h"
#include "board.h"

#include "stm32g4xx_hal.h"

void board_init(void) {
    board_led_write(false);

    HAL_NVIC_SetPriority(USB_HP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_HP_IRQn);

    HAL_NVIC_SetPriority(USB_LP_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);

    HAL_NVIC_SetPriority(USBWakeUp_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USBWakeUp_IRQn);
}

int board_uart_write(void const *buf, int len) {
    (void) buf;
    return len;
}

uint32_t board_millis(void) {
    return HAL_GetTick();
}

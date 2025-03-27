/*
 * Misc.c
 *
 *  Created on: Mar 26, 2025
 *      Author: Thomas
 */

#include "main.h"
#include "Misc.h"

void send_uart(char *string) {
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}

void send_byte_as_binary(uint8_t byte) {
    char binaryString[9];  // 8 bits + null terminator

    for (int i = 0; i < 8; i++) {
        binaryString[i] = (byte & (1 << (7 - i))) ? '1' : '0';
    }
    binaryString[8] = '\0';  // Null-terminate the string

    send_uart(binaryString); // Send the binary string
    send_uart("\r\n");       // Newline for readability
}

void delay_ns(uint32_t ns) {
    uint32_t cycles_per_ns = SystemCoreClock / 1000000000; // Convert clock speed to cycles per ns
    uint32_t start = DWT->CYCCNT;                         // Get start cycle count
    uint32_t delay_cycles = ns * cycles_per_ns;           // Calculate required cycles

    while ((DWT->CYCCNT - start) < delay_cycles);         // Wait until delay is met
}

void USART1_Printf(const char *format, ...) {
    char buffer[128];  // Adjust size as needed
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

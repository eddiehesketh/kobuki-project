#pragma once
#include <stdint.h>

void UART_Init(void);
uint8_t UART_Rx(void);
void UART_Tx(uint8_t byte);
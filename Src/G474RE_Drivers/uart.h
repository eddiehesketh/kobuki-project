// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        uart.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Polled USART3 driver (PC10 TX, PC11 RX, 115200 baud) used to
//              talk to the Kobuki.
// FSM:         None. Transport for kobuki.c.
// Provenance:  Provided – practical drivers.
// ============================================================================

#pragma once

#include <stdint.h>

// UART_Init
// Provenance: Provided – practical drivers.
// Purpose:    Set up USART3 on PC10/PC11 at 115200 baud from HSI16.
// Params:     none.
// Returns:    nothing.
// Effects:    enables the GPIOC and USART3 clocks; configures the pins and USART3.
void UART_Init(void);

// UART_Rx
// Provenance: Provided – practical drivers.
// Purpose:    Receive one byte.
// Params:     none.
// Returns:    the received byte.
// Effects:    clears an overrun error; blocks until a byte arrives.
uint8_t UART_Rx(void);

// UART_Tx
// Provenance: Provided – practical drivers.
// Purpose:    Send one byte.
// Params:     byte - byte to send.
// Returns:    nothing.
// Effects:    blocks until the transmit register is free.
void UART_Tx(uint8_t byte);

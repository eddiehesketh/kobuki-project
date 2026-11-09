// ============================================================================
// File:        uart.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Polled USART3 driver (PC10 TX, PC11 RX, 115200 baud) used to
//              talk to the Kobuki.
// FSM:         None. Transport for kobuki.c.
// Provenance:  Provided – practical drivers.
// ============================================================================

#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"

#define HSI16_HZ 16000000U // USART3 clock source (Hz)
#define KOBUKI_BAUD_RATE 115200U // Kobuki serial rate (baud)
// Baud divider, rounded to the nearest whole number: 16 MHz / 115200 = 139.
#define USART3_BRR_VALUE ((HSI16_HZ + (KOBUKI_BAUD_RATE / 2U)) / KOBUKI_BAUD_RATE)
#define USART3_AF 7U // PC10/PC11 alternate function for USART3

// Provided – practical drivers.
void UART_Init(void)
{
    // Enable peripheral clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;

    // Route HSI16 clock to USART3
    RCC->CCIPR &= ~RCC_CCIPR_USART3SEL;
    RCC->CCIPR |= RCC_CCIPR_USART3SEL_1; // 0b10: HSI16

    // Configure PC10 (TX) and PC11 (RX)
    GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
    GPIOC->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

    // Alternate function mode
    // Route Alternate Function 7 (AF7) to PC10 and PC11
    GPIOC->AFR[1] &= ~(GPIO_AFRH_AFSEL10 | GPIO_AFRH_AFSEL11);
    GPIOC->AFR[1] |= ((USART3_AF << GPIO_AFRH_AFSEL10_Pos) | (USART3_AF << GPIO_AFRH_AFSEL11_Pos));

    // Configure USART3
    USART3->CR1 &= ~USART_CR1_UE; // Turn off to allow config

    // Set Baud Rate to 115200 from the 16 MHz HSI16 clock.
    USART3->BRR = USART3_BRR_VALUE;

    // Enable Transmitter (TE) and Receiver (RE)
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Enable USART3 (UE)
    USART3->CR1 |= USART_CR1_UE;
}

// Provided – practical drivers.
void UART_Tx(uint8_t byte)
{
    // Wait until Transmit Data Register is Empty (TXE) flag is set
    while (!(USART3->ISR & USART_ISR_TXE)) {
    }

    // Write the byte to the Transmit Data Register
    USART3->TDR = byte;
}

// Provided – practical drivers.
uint8_t UART_Rx(void)
{
    // If an Overrun Error (ORE) occurs, clear it so we don't get permanently stuck
    if (USART3->ISR & USART_ISR_ORE) {
        USART3->ICR = USART_ICR_ORECF; // Write 1 to clear the Overrun flag
    }

    // No timeout: the Kobuki streams feedback every 20 ms, so this wait is
    // bounded in normal operation and paces the main loop at 50 Hz.
    while (!(USART3->ISR & USART_ISR_RXNE)) {
    }

    // Reading RDR automatically clears the RXNE flag
    return (uint8_t)(USART3->RDR & 0xFF);
}

// ============================================================================
// File:        adc.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Polled ADC1 driver for single 12-bit conversions on regular
//              channels 1 to 18.
// FSM:         None. Used by ultrasonic.c.
// Provenance:  Provided – practical drivers.
// ============================================================================

#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "adc.h"

// Provided – practical drivers.
void ADC_Init(void)
{
    // Enable the ADC12 peripheral clock.
    RCC->AHB2ENR |= (1 << 13);

    // Select synchronous HCLK/1 as the ADC clock.
    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0;

    // Exit deep-power-down mode and enable the ADC voltage regulator.
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;

    // Allow the voltage regulator to stabilise.
    for (volatile uint32_t i = 0U; i < 1000U; ++i) {
        __NOP();
    }

    // Calibrate ADC1 in single-ended mode.
    // This busy-wait and the one below are bounded by the hardware:
    // calibration and enable each finish in a fixed number of ADC clocks.
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->CR & ADC_CR_ADCAL) != 0U) {
    }

    // Enable ADC1 and wait until it is ready.
    ADC1->ISR = ADC_ISR_ADRDY; // Clear the ready flag
    ADC1->CR |= ADC_CR_ADEN; // Enable the ADC

    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U) {
    }
}

// Provided – practical drivers.
uint16_t ADC_Read(uint8_t channel)
{
    // This driver supports ADC1 regular channels 1 to 18.
    if ((channel < 1U) || (channel > 18U)) {
        return 0U;
    }

    // Select one regular conversion and place the channel in SQ1.
    ADC1->SQR1 = ((uint32_t)channel << ADC_SQR1_SQ1_Pos);

    // Select the longest sample time for a stable sensor measurement.
    if (channel <= 9U) {
        uint32_t position = (uint32_t)channel * 3U;
        ADC1->SMPR1 &= ~(7UL << position);
        ADC1->SMPR1 |= (7UL << position);
    } else {
        uint32_t position = ((uint32_t)channel - 10U) * 3U;
        ADC1->SMPR2 &= ~(7UL << position);
        ADC1->SMPR2 |= (7UL << position);
    }

    // Clear completion flags and start the conversion.
    // The wait is bounded by the hardware: one conversion takes a fixed number of ADC clocks.
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {
    }

    return (uint16_t)(ADC1->DR & ADC_DR_RDATA);
}

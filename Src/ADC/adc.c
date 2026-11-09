// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
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

#define ADC_REGULATOR_DELAY_LOOPS 1000U // busy-wait for the voltage regulator to settle
#define ADC_FIRST_CHANNEL 1U // lowest regular channel this driver accepts
#define ADC_LAST_CHANNEL 18U // highest regular channel this driver accepts
#define ADC_SMPR1_LAST_CHANNEL 9U // channels 0-9 are in SMPR1, 10-18 in SMPR2
#define ADC_SMPR2_FIRST_CHANNEL 10U
#define ADC_SMP_BITS_PER_CHANNEL 3U // sample-time field width
#define ADC_SMP_LONGEST 7UL // 0b111: longest sample time (640.5 ADC clocks)

// Provided – practical drivers.
void ADC_Init(void)
{
    // Enable the ADC12 peripheral clock.
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

    // Select synchronous HCLK/1 as the ADC clock.
    ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
    ADC12_COMMON->CCR |= ADC_CCR_CKMODE_0;

    // Exit deep-power-down mode and enable the ADC voltage regulator.
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;

    // Allow the voltage regulator to stabilise.
    for (volatile uint32_t i = 0U; i < ADC_REGULATOR_DELAY_LOOPS; ++i) {
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
    if ((channel < ADC_FIRST_CHANNEL) || (channel > ADC_LAST_CHANNEL)) {
        return 0U;
    }

    // Select one regular conversion and place the channel in SQ1.
    ADC1->SQR1 = ((uint32_t)channel << ADC_SQR1_SQ1_Pos);

    // Select the longest sample time for a stable sensor measurement.
    if (channel <= ADC_SMPR1_LAST_CHANNEL) {
        uint32_t position = (uint32_t)channel * ADC_SMP_BITS_PER_CHANNEL;
        ADC1->SMPR1 &= ~(ADC_SMP_LONGEST << position);
        ADC1->SMPR1 |= (ADC_SMP_LONGEST << position);
    } else {
        uint32_t position = ((uint32_t)channel - ADC_SMPR2_FIRST_CHANNEL) * ADC_SMP_BITS_PER_CHANNEL;
        ADC1->SMPR2 &= ~(ADC_SMP_LONGEST << position);
        ADC1->SMPR2 |= (ADC_SMP_LONGEST << position);
    }

    // Clear completion flags and start the conversion.
    // The wait is bounded by the hardware: one conversion takes a fixed number of ADC clocks.
    ADC1->ISR = ADC_ISR_EOC | ADC_ISR_EOS;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {
    }

    return (uint16_t)(ADC1->DR & ADC_DR_RDATA);
}

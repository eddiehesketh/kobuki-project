// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        ultrasonic.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: MB1043 ultrasonic sensor on PA0 (ADC1 channel 1). Converts the
//              analog output to a range in mm and flags a detection.
// FSM:         Sensor input for FSM 1, FSM 2 and FSM 3.
// Provenance:  Provided - practical drivers.
// ============================================================================

#include <stdint.h>

#define STM32G474xx
#include "stm32g4xx.h"
#include "adc.h"
#include "ultrasonic.h"

#define ULTRASONIC_CHANNEL ADC_CHANNEL_1
#define ADC_MAX_VALUE 4095UL // 12-bit ADC full scale
#define MB1043_FULL_SCALE_MM 5120UL // MB1043 output is Vcc / 5120 per mm

// Provided - practical drivers.
void Ultrasonic_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODE0; // analog mode
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0; // no pull-up or pull-down
}

// Provided - practical drivers.
uint16_t Ultrasonic_Read(void)
{
    uint16_t adc_value = ADC_Read(ULTRASONIC_CHANNEL);
    // range = adc × 5120 / 4095, rounded to the nearest mm.
    uint32_t distance_mm = ((uint32_t)adc_value * MB1043_FULL_SCALE_MM
        + (ADC_MAX_VALUE / 2UL)) / ADC_MAX_VALUE;

    if (distance_mm > ULTRASONIC_MAX_RANGE_MM) {
        distance_mm = ULTRASONIC_MAX_RANGE_MM;
    }

    return (uint16_t)distance_mm;
}

// Provided - practical drivers.
void Ultrasonic_Detect(US_Typedef *object)
{
    if (object == 0) {
        return;
    }

    object->range_mm = Ultrasonic_Read();
    object->detected = (object->range_mm < ULTRASONIC_DETECT_RANGE_MM) ? 1U : 0U;
}

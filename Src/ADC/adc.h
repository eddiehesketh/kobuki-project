// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        adc.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Polled ADC1 driver for single 12-bit conversions on regular
//              channels 1 to 18.
// FSM:         None. Used by ultrasonic.c.
// Provenance:  Provided - practical drivers.
// ============================================================================

#pragma once

#include <stdint.h>

#define ADC_CHANNEL_1 ((uint8_t)1U) // ADC1_IN1 on PA0

// ADC_Init
// Provenance: Provided - practical drivers.
// Purpose:    Power up, calibrate and enable ADC1.
// Params:     none.
// Returns:    nothing.
// Effects:    enables the ADC12 clock; blocks until calibration and the ready
//             flag finish (bounded by the hardware).
void ADC_Init(void);

// ADC_Read
// Provenance: Provided - practical drivers.
// Purpose:    Run one conversion on an ADC1 regular channel.
// Params:     channel - channel number, 1 to 18.
// Returns:    12-bit result (0 to 4095), or 0 for an invalid channel.
// Effects:    sets the channel's sample time; blocks until the conversion ends
//             (bounded by the hardware).
uint16_t ADC_Read(uint8_t channel);

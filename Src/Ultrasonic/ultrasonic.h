// ============================================================================
// File:        ultrasonic.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: MB1043 ultrasonic sensor on PA0 (ADC1 channel 1). Converts the
//              analog output to a range in mm and flags a detection.
// FSM:         Sensor input for FSM 1, FSM 2 and FSM 3.
// Provenance:  Provided – practical drivers.
// ============================================================================

#pragma once

#include <stdint.h>

// Ceiling only. Each caller applies its own tighter max range.
// mm. Raised from 700 mm so bollard A1 (610-840 mm) is detected (report 5.3).
#define ULTRASONIC_DETECT_RANGE_MM ((uint16_t)1000U)
// mm. MB1043 maximum reported range.
#define ULTRASONIC_MAX_RANGE_MM ((uint16_t)5000U)

typedef struct {
    uint8_t detected; // 1 if range_mm < ULTRASONIC_DETECT_RANGE_MM
    uint16_t range_mm; // measured range (mm)
} US_Typedef;

// Ultrasonic_Init
// Provenance: Provided – practical drivers.
// Purpose:    Set PA0 to analog input for the sensor.
// Params:     none.
// Returns:    nothing.
// Effects:    enables the GPIOA clock; configures PA0.
void Ultrasonic_Init(void);

// Ultrasonic_Read
// Provenance: Provided – practical drivers.
// Purpose:    Read the sensor range.
// Params:     none.
// Returns:    range (mm), 0 to ULTRASONIC_MAX_RANGE_MM.
// Effects:    runs one ADC conversion.
uint16_t Ultrasonic_Read(void);

// Ultrasonic_Detect
// Provenance: Provided – practical drivers.
// Purpose:    Read the range and flag whether an object is in detection range.
// Params:     object - result to fill (ignored if null).
// Returns:    nothing.
// Effects:    runs one ADC conversion; writes object->range_mm and object->detected.
void Ultrasonic_Detect(US_Typedef *object);

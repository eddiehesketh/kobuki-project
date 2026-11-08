// ============================================================================
// File:        orient.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Scan-and-face. Runs the edge scan (FSM 3) with a range limit,
//              then turns to face the object it found. Used for the start
//              bollard (840 mm) and the deposit marker (560 mm).
// FSM:         FSM 2 – scan-and-face (report 4.2). Used by ORIENT and STORAGE in FSM 1.
// Provenance:  Original.
// ============================================================================

#pragma once

#include "navigation.h"
#include "ultrasonic.h"

// Orient_Update return values.
#define ORIENT_RUNNING 0U // scanning or turning
#define ORIENT_DONE 1U // facing the object
#define ORIENT_FAIL 2U // nothing found within the sweep limit

// FSM 2: Scan-and-face (report 4.2). Used by ORIENT and STORAGE in FSM 1.
typedef enum {
    ORIENT_IDLE,
    ORIENT_SCANNING,
    ORIENT_TURNING
} OrientStep;

extern OrientStep orient_step;

// Orient_Update
// Provenance: Original.
// Purpose:    One step of FSM 2.
// Params:     kobuki            - current robot state.
//             ultrasonic        - latest ultrasonic reading.
//             navigation_object - FSM 3 scan state.
//             max_range_mm      - objects beyond this range (mm) are ignored.
// Returns:    ORIENT_RUNNING while orienting, ORIENT_DONE when the turn
//             finishes, ORIENT_FAIL when the scan fails.
// Effects:    sends rotate commands; stops the motors and returns orient_step
//             to ORIENT_IDLE on ORIENT_DONE or ORIENT_FAIL.
uint8_t Orient_Update(Kobuki_Typedef *kobuki, US_Typedef *ultrasonic,
                      NavigationScan *navigation_object, uint16_t max_range_mm);

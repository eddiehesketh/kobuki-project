// ============================================================================
// File:        navigation.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Edge scan. Spins counter-clockwise, records the heading where
//              an object enters (theta 1) and leaves (theta 2) the ultrasonic
//              beam, then turns to face the lag-corrected midpoint.
// FSM:         FSM 3 – edge scan (report 4.3). Run by ORIENT_SCANNING in FSM 2.
// Provenance:  Original.
// ============================================================================

#pragma once
#include "kobuki.h"
#include <stdint.h>

// FSM 3: Edge scan (report 4.3). Run by ORIENT_SCANNING in FSM 2.
typedef enum {
    CLEAR_OBJECT = 0,
    FIND_ANGLE_1,
    FIND_ANGLE_2,
    SCAN_COMPLETE,
    SCAN_FAILED
} NavigationScanState;

typedef struct {
    NavigationScanState state;

    int16_t angle1; // theta 1: object enters the beam
    int16_t angle2; // theta 2: object leaves the beam
    int16_t midpoint; // theta mid: heading to face

    int16_t previous_angle; // heading at the last step (gyro ticks, 0.01 deg)
    int32_t swept_angle; // total turn in the current state (gyro ticks, 0.01 deg)

    // An edge counts after SCAN_DEBOUNCE_SAMPLES readings. The saved angle is the first.
    uint8_t run_count;
    int16_t run_start_angle;
} NavigationScan;

// Navigation_Midpoint
// Provenance: Original.
// Purpose:    Heading halfway along the short arc between two headings.
// Params:     first_angle, second_angle - headings (gyro ticks, 0.01 deg).
// Returns:    midpoint heading in -18000..17999 (gyro ticks, 0.01 deg).
// Effects:    none.
// Maths:      θmid = θ1 + wrap(θ2 − θ1) / 2.
int16_t Navigation_Midpoint(int16_t first_angle, int16_t second_angle);

// Navigation_Scan_Start
// Provenance: Original.
// Purpose:    Reset FSM 3 and pick its first state.
// Params:     scan          - scan state to reset.
//             current_angle - gyro heading (gyro ticks, 0.01 deg).
//             detected      - 1 if an object is in range now.
// Returns:    nothing.
// Effects:    clears the saved angles and counters; starts in CLEAR_OBJECT if
//             an object is already in the beam, else FIND_ANGLE_1.
void Navigation_Scan_Start(NavigationScan *scan, int16_t current_angle, uint8_t detected);

// Navigation_Scan_Update
// Provenance: Original.
// Purpose:    One step of FSM 3.
// Params:     scan          - scan state.
//             current_angle - gyro heading (gyro ticks, 0.01 deg).
//             detected      - 1 if an object is in range.
// Returns:    nothing. Read scan->state: SCAN_COMPLETE once the midpoint is
//             found, SCAN_FAILED once the sweep limit is reached.
// Effects:    sends a rotate command at SCAN_SPEED_MM_S (mm/s); stops on
//             completion or failure.
// Maths:      θmid = θ1 + wrap(θ2 − θ1) / 2 − 5.5°; sweep limit 360° (180° in FIND_ANGLE_2).
void Navigation_Scan_Update(NavigationScan *scan, int16_t current_angle, uint8_t detected);

// Navigation_Face_Start
// Provenance: Original.
// Purpose:    Start the turn to the scan midpoint. Used by ORIENT_TURNING in FSM 2.
// Params:     target_angle - heading to face (gyro ticks, 0.01 deg).
// Returns:    nothing.
// Effects:    sets a rotation setpoint; clears rotation_complete.
void Navigation_Face_Start(int16_t target_angle); // turn to the bollard midpoint

// Navigation_Face_Update
// Provenance: Original.
// Purpose:    One step of the turn to the scan midpoint.
// Params:     kobuki - current robot state.
// Returns:    1 when the turn finishes, else 0.
// Effects:    sends a rotate command; stops on completion.
uint8_t Navigation_Face_Update(const Kobuki_Typedef *kobuki); // 1 when the turn finishes

// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        navigation.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Edge scan. Spins counter-clockwise, records the heading where
//              an object enters (theta 1) and leaves (theta 2) the ultrasonic
//              beam, then turns to face the lag-corrected midpoint.
// FSM:         FSM 3 - edge scan (report 4.3). Run by ORIENT_SCANNING in FSM 2.
// Provenance:  Original.
// ============================================================================

#include "navigation.h"
#include "kobuki.h"

// Scan wheel speed (mm/s). 75 mm/s turns the robot about 37°/s (report Appendix B).
#define SCAN_SPEED_MM_S ((int16_t)75)

// Readings in a row before an edge counts. 3 samples is about 60 ms at 50 Hz.
#define SCAN_DEBOUNCE_SAMPLES ((uint8_t)3U)

// MB1043 lags ~150 ms, so both edges are late and the midpoint overshoots.
// At 75 mm/s that is about 5.5 deg. Units are 1/100 deg. Tune on the robot.
// 550 = 37°/s × 0.15 s sensor lag.
#define SCAN_LAG_COMP_CENTIDEG ((int32_t)550)

// Original.
int16_t Navigation_Midpoint(int16_t first_angle, int16_t second_angle)
{
    int32_t difference = (int32_t)second_angle - first_angle;

    // Shortest arc. Units are 1/100 deg.
    if (difference > HALF_TURN_CENTIDEG) {
        difference -= FULL_TURN_CENTIDEG;
    } else if (difference < -HALF_TURN_CENTIDEG) {
        difference += FULL_TURN_CENTIDEG;
    }

    int32_t midpoint = first_angle + difference / 2;

    // Keep the result in -18000..17999.
    if (midpoint >= HALF_TURN_CENTIDEG) {
        midpoint -= FULL_TURN_CENTIDEG;
    } else if (midpoint < -HALF_TURN_CENTIDEG) {
        midpoint += FULL_TURN_CENTIDEG;
    }

    return (int16_t)midpoint;
}

// Original.
// Reset FSM 3 and pick its first state.
void Navigation_Scan_Start(NavigationScan *scan, int16_t current_angle, uint8_t detected)
{
    scan->angle1 = 0;
    scan->angle2 = 0;
    scan->midpoint = 0;
    scan->previous_angle = current_angle;
    scan->swept_angle = 0;
    scan->run_count = 0U;
    scan->run_start_angle = current_angle;

    // Already on an object: find a clear direction first.
    scan->state = (detected != 0U) ? CLEAR_OBJECT : FIND_ANGLE_1;
}

// Original.
// FSM 3: Edge scan. Rotates counter-clockwise until both edges are found.
void Navigation_Scan_Update(NavigationScan *scan, int16_t current_angle, uint8_t detected)
{
    if ((scan->state == SCAN_COMPLETE) || (scan->state == SCAN_FAILED)) {
        Kobuki_Rotate(0);
        return;
    }

    int32_t change = (int32_t)current_angle - scan->previous_angle;

    if (change > HALF_TURN_CENTIDEG) {
        change -= FULL_TURN_CENTIDEG;
    } else if (change < -HALF_TURN_CENTIDEG) {
        change += FULL_TURN_CENTIDEG;
    }

    scan->previous_angle = current_angle;
    scan->swept_angle += (change >= 0) ? change : -change;

    // Second edge is limited to 180 deg. The first search may use a full turn.
    int32_t limit = (scan->state == FIND_ANGLE_2) ? HALF_TURN_CENTIDEG : FULL_TURN_CENTIDEG;

    if (scan->swept_angle >= limit) {
        scan->state = SCAN_FAILED;
        Kobuki_Rotate(0);
        return;
    }

    // FSM 3: edge scan. The robot spins counter-clockwise in every running state.
    //   CLEAR_OBJECT  - started facing an object. Next: FIND_ANGLE_1 on the
    //                   first clear reading.
    //   FIND_ANGLE_1  - wait for an object to enter the beam. Next: FIND_ANGLE_2
    //                   after 3 detections in a row; theta 1 = the first one.
    //   FIND_ANGLE_2  - wait for the object to leave. Next: SCAN_COMPLETE after
    //                   3 clear readings in a row; theta 2 = the first one, and
    //                   the lag-corrected midpoint is saved.
    //   SCAN_COMPLETE - stopped; midpoint ready.
    //   SCAN_FAILED   - stopped. Entered from any running state once the sweep
    //                   limit is reached (360°, or 180° in FIND_ANGLE_2).
    switch (scan->state) {
        case CLEAR_OBJECT: // wait for the starting object to leave the beam
            // Not debounced, so one dropout could end it early. Left as is: the
            // robot rarely starts facing an object (report 4.7, 5.5).
            if (detected == 0U) {
                scan->state = FIND_ANGLE_1;
                scan->swept_angle = 0; // this step gets its own angle limit
            }
            break;

        case FIND_ANGLE_1: // wait for the object to enter: theta 1
            if (detected != 0U) {
                if (scan->run_count == 0U) {
                    scan->run_start_angle = current_angle; // first sample of the run
                }
                scan->run_count++;
                if (scan->run_count >= SCAN_DEBOUNCE_SAMPLES) {
                    scan->angle1 = scan->run_start_angle; // where detection began
                    scan->state = FIND_ANGLE_2;
                    scan->swept_angle = 0;
                    scan->run_count = 0U;
                }
            } else {
                scan->run_count = 0U; // one-off blip, ignore it
            }
            break;

        case FIND_ANGLE_2: // wait for the object to leave: theta 2, then the midpoint
            if (detected == 0U) {
                if (scan->run_count == 0U) {
                    scan->run_start_angle = current_angle;
                }
                scan->run_count++;
                if (scan->run_count >= SCAN_DEBOUNCE_SAMPLES) {
                    scan->angle2 = scan->run_start_angle;
                    int32_t mid = Navigation_Midpoint(scan->angle1, scan->angle2);

                    // Undo sensor lag: shift back against the scan direction.
                    mid += (SCAN_SPEED_MM_S >= 0) ? -SCAN_LAG_COMP_CENTIDEG : SCAN_LAG_COMP_CENTIDEG;
                    if (mid >= HALF_TURN_CENTIDEG) {
                        mid -= FULL_TURN_CENTIDEG;
                    } else if (mid < -HALF_TURN_CENTIDEG) {
                        mid += FULL_TURN_CENTIDEG;
                    }

                    scan->midpoint = (int16_t)mid;
                    scan->state = SCAN_COMPLETE;
                    Kobuki_Rotate(0);
                    return;
                }
            } else {
                scan->run_count = 0U; // dropout mid-bollard, keep going
            }
            break;

        case SCAN_COMPLETE:
            break;

        case SCAN_FAILED:
            break;
    }

    Kobuki_Rotate(SCAN_SPEED_MM_S);
}

// Original.
// Used by ORIENT_TURNING in FSM 2.
void Navigation_Face_Start(int16_t target_angle)
{
    Kobuki_Rotate_Angle_Setpoint(target_angle, 0);
}

// Original.
// 1 when the turn finishes.
uint8_t Navigation_Face_Update(const Kobuki_Typedef *kobuki)
{
    if (kobuki->rotation_complete != 0U) {
        Kobuki_Rotate(0);
        return 1U;
    }

    Kobuki_Rotate_Angle(kobuki->angle);
    return 0U;
}

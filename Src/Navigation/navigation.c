#include "navigation.h"
#include "kobuki.h"

#define SCAN_SPEED_MM_S ((int16_t)75)

// Readings in a row before an edge counts. 3 samples is about 60 ms at 50 Hz.
#define SCAN_DEBOUNCE_SAMPLES ((uint8_t)3U)

// MB1043 lags ~150 ms, so both edges are late and the midpoint overshoots.
// At 75 mm/s that is about 5.5 deg. Units are 1/100 deg. Tune on the robot.
#define SCAN_LAG_COMP_CENTIDEG ((int32_t)550)

int16_t Navigation_Midpoint(int16_t first_angle, int16_t second_angle)
{
    int32_t difference = (int32_t)second_angle - first_angle;

    // Shortest arc. Units are 1/100 deg.
    if (difference > 18000) {
        difference -= 36000;
    }
    else if (difference < -18000) {
        difference += 36000;
    }

    int32_t midpoint = first_angle + difference / 2;

    // Keep the result in -18000..17999.
    if (midpoint >= 18000) {
        midpoint -= 36000;
    }
    else if (midpoint < -18000) {
        midpoint += 36000;
    }

    return (int16_t)midpoint;
}

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

// FSM 3: Edge scan. Rotates counter-clockwise until both edges are found.
void Navigation_Scan_Update(NavigationScan *scan, int16_t current_angle, uint8_t detected)
{
    if ((scan->state == SCAN_COMPLETE) || (scan->state == SCAN_FAILED)) {
        Kobuki_Rotate(0);
        return;
    }

    int32_t change = (int32_t)current_angle - scan->previous_angle;

    if (change > 18000) {
        change -= 36000;
    }
    else if (change < -18000) {
        change += 36000;
    }

    scan->previous_angle = current_angle;
    scan->swept_angle += (change >= 0) ? change : -change;

    // Second edge is limited to 180 deg. The first search may use a full turn.
    int32_t limit = (scan->state == FIND_ANGLE_2) ? 18000 : 36000;

    if (scan->swept_angle >= limit) {
        scan->state = SCAN_FAILED;
        Kobuki_Rotate(0);
        return;
    }

    switch (scan->state) {
        case CLEAR_OBJECT: // wait for the starting object to leave the beam
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
            }
            else {
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
                    if (mid >= 18000) {
                        mid -= 36000;
                    }
                    else if (mid < -18000) {
                        mid += 36000;
                    }

                    scan->midpoint = (int16_t)mid;
                    scan->state = SCAN_COMPLETE;
                    Kobuki_Rotate(0);
                    return;
                }
            }
            else {
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

// Used by ORIENT_TURNING in FSM 2.
void Navigation_Orient_Start(int16_t target_angle)
{
    Kobuki_Rotate_Angle_Setpoint(target_angle, 0);
}

// 1 when the turn finishes.
uint8_t Navigation_Orient_Update(const Kobuki_Typedef *kobuki)
{
    if (kobuki->rotation_complete != 0U) {
        Kobuki_Rotate(0);
        return 1U;
    }

    Kobuki_Rotate_Angle(kobuki->angle);
    return 0U;
}

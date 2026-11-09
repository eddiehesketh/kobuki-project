// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        orient.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Scan-and-face. Runs the edge scan (FSM 3) with a range limit,
//              then turns to face the object it found. Used for the start
//              bollard (840 mm) and the deposit marker (560 mm).
// FSM:         FSM 2 - scan-and-face (report 4.2). Used by ORIENT and STORAGE in FSM 1.
// Provenance:  Original.
// ============================================================================

#include "orient.h"

OrientStep orient_step = ORIENT_IDLE;

// Only count an object inside this scan's max range (mm): 1 if in range, else 0.
static uint8_t detection_in_range(US_Typedef *ultrasonic, uint16_t max_range_mm)
{
    if ((ultrasonic->detected == 0U) || (ultrasonic->range_mm > max_range_mm)) {
        return 0U;
    }

    return 1U;
}

// Original.
// FSM 2: Scan-and-face.
uint8_t Orient_Update(Kobuki_Typedef *kobuki, US_Typedef *ultrasonic,
                      NavigationScan *navigation_object, uint16_t max_range_mm)
{
    uint8_t detected = detection_in_range(ultrasonic, max_range_mm);

    // FSM 2: scan-and-face.
    //   ORIENT_IDLE     - set up FSM 3. Next: ORIENT_SCANNING.
    //   ORIENT_SCANNING - one step of FSM 3 per loop. Next: ORIENT_TURNING on
    //                     SCAN_COMPLETE; ORIENT_IDLE and ORIENT_FAIL on SCAN_FAILED.
    //   ORIENT_TURNING  - turn to the scan midpoint. Next: ORIENT_IDLE and
    //                     ORIENT_DONE once the turn finishes.
    switch (orient_step) {
        case ORIENT_IDLE: // set up FSM 3
            Navigation_Scan_Start(navigation_object, kobuki->angle, detected);
            orient_step = ORIENT_SCANNING;
            break;
        case ORIENT_SCANNING: // one step of FSM 3
            Navigation_Scan_Update(navigation_object, kobuki->angle, detected);
            if (navigation_object->state == SCAN_COMPLETE) {
                Navigation_Face_Start(navigation_object->midpoint);
                orient_step = ORIENT_TURNING;
            } else if (navigation_object->state == SCAN_FAILED) {
                Kobuki_Drive(0);
                orient_step = ORIENT_IDLE;
                return ORIENT_FAIL;
            }
            break;
        case ORIENT_TURNING: // turn to the scan midpoint
            if (Navigation_Face_Update(kobuki) != 0U) {
                Kobuki_Drive(0);
                orient_step = ORIENT_IDLE;
                return ORIENT_DONE;
            }
            break;
        default:
            Kobuki_Drive(0);
            orient_step = ORIENT_IDLE;
            return ORIENT_FAIL;
    }

    return ORIENT_RUNNING;
}

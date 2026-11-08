#include "orient.h"

OrientStep orient_step = ORIENT_IDLE;

// Only count an object inside this scan's max range.
static uint8_t detection_in_range(US_Typedef *ultrasonic, uint16_t max_range_mm)
{
    if ((ultrasonic->detected == 0U) || (ultrasonic->range_mm > max_range_mm)) {
        return 0U;
    }

    return 1U;
}

// FSM 2: Scan-and-face.
uint8_t orient_fsm(Kobuki_Typedef *kobuki, US_Typedef *ultrasonic,
                   NavigationScan *navigation_object, uint16_t max_range_mm)
{
    uint8_t detected = detection_in_range(ultrasonic, max_range_mm);

    switch (orient_step) {
        case ORIENT_IDLE: // set up FSM 3
            Navigation_Scan_Start(navigation_object, kobuki->angle, detected);
            orient_step = ORIENT_SCANNING;
            break;
        case ORIENT_SCANNING: // one step of FSM 3
            Navigation_Scan_Update(navigation_object, kobuki->angle, detected);
            if (navigation_object->state == SCAN_COMPLETE) {
                Navigation_Orient_Start(navigation_object->midpoint);
                orient_step = ORIENT_TURNING;
            }
            else if (navigation_object->state == SCAN_FAILED) {
                Kobuki_Drive(0);
                orient_step = ORIENT_IDLE;
                return 2U;
            }
            break;
        case ORIENT_TURNING: // turn to the scan midpoint
            if (Navigation_Orient_Update(kobuki) != 0U) {
                Kobuki_Drive(0);
                orient_step = ORIENT_IDLE;
                return 1U;
            }
            break;
        default:
            Kobuki_Drive(0);
            orient_step = ORIENT_IDLE;
            return 2U;
    }

    return 0U;
}

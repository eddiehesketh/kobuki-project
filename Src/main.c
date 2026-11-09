// ============================================================================
// File:        main.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Entry point and main state machine. Faces the start bollard,
//              picks the layout from its range, drives the path into storage,
//              then finds the deposit marker and stops 100 mm in front of it.
// FSM:         FSM 1 – main state machine (report 4.1). Runs FSM 2, FSM 4
//              and FSM 5.
// Provenance:  Original.
// ============================================================================

#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "adc.h"
#include "kobuki.h"
#include "uart.h"
#include "ultrasonic.h"
#include "pathfind.h"
#include "navigation.h"
#include "orient.h"
#include "decision.h"
#include "reflex.h"

// Kobuki button bits (kobuki.button).
#define BUTTON_0_START 0x01U // B0: start a run
#define BUTTON_1_STOP 0x02U // B1: stop, back to IDLE

// Orient scan stops at the far edge of layout A.
#define ORIENT_MAX_RANGE_MM LAYOUT_A_MAX_MM
// mm. Between the deposit marker (~377 mm) and bollard B2 (~663 mm) from the
// end of leg 3; lowered from 620 mm so B2 is ignored (report 3.8, 5.3).
#define STORAGE_MAX_RANGE_MM 560 // mm
// mm. Stop distance from the marker. 100 mm overshot, so 150 mm was set on
// the course to give the required 100 mm stop (report 3.8).
#define FINISHING_DISTANCE_MM 150

// FSM 1: Main state machine (report 4.1).
typedef enum {
    STATE_IDLE,
    STATE_ORIENT,
    STATE_DECISION,
    STATE_ALIGN,
    STATE_PATHFIND,
    STATE_STORAGE,
    STATE_DEPOSIT
} MainState;

// Original. FSM 1 – main state machine.
// Purpose: Set up the peripherals, then run FSM 1 forever.
// Params:  none.
// Returns: never returns.
// Effects: drives the robot through a full run each time B0 is pressed.
int main(void)
{
    MainState current_state = STATE_IDLE;

    Kobuki_Typedef kobuki = {0};
    US_Typedef ultrasonic = {0};
    NavigationScan navigation_object;

    UART_Init();
    ADC_Init();
    Ultrasonic_Init();

    // Each loop: safety check, deposit arrival, FSM 5, then one step of FSM 1.
    // Kobuki_Read waits for the next feedback packet, so the loop runs at 50 Hz.
    while (1) {
        Kobuki_Read(&kobuki);
        Ultrasonic_Detect(&ultrasonic);

        // 1. Safety check.
        // Wheel drop or the stop button overrides the FSM.
        if ((kobuki.wheeldrop != 0U) ||
            ((kobuki.button & BUTTON_1_STOP) != 0U)) {
            Kobuki_Drive(0);
            Pathfind_Reset();
            Reflex_Reset();
            orient_step = ORIENT_IDLE;
            current_state = STATE_IDLE;
            continue;
        }

        // Bumper and cliff recovery only on PATHFIND drive legs.
        uint8_t driving_forward =
            (current_state == STATE_PATHFIND) &&
            ((path_state == FIRST_DRIVE) ||
             (path_state == SECOND_DRIVE) ||
             (path_state == THIRD_DRIVE));

        // 2. Deposit arrival.
        // In DEPOSIT a bumper means the final bollard was reached.
        // This must run before FSM 5 and the switch: it pre-empts FSM 5, which
        // would otherwise treat the arrival bump as a collision.
        if ((current_state == STATE_DEPOSIT) && (kobuki.bumper != 0U) &&
            (kobuki.cliff == 0U) && (Reflex_Busy() == 0U)) {
            Kobuki_Drive(0);
            current_state = STATE_IDLE;
            continue;
        }

        // 3. FSM 5: bump and cliff recovery.
        uint8_t reflex_result = Reflex_Update(&kobuki, driving_forward);
        if (reflex_result == REFLEX_GAVE_UP) { // seventh hit
            Kobuki_Drive(0);
            Pathfind_Reset();
            Reflex_Reset();
            orient_step = ORIENT_IDLE;
            current_state = STATE_IDLE;
            continue;
        }
        if (reflex_result == REFLEX_ACTIVE) {
            continue; // recovery owns the motors this loop
        }

        // 4. One step of the current state.
        // FSM 1: main state machine.
        //   STATE_IDLE     - motors off, sub-FSMs reset. Next: STATE_ORIENT when
        //                    B0 alone is pressed.
        //   STATE_ORIENT   - FSM 2 at 840 mm: face the start bollard. Next:
        //                    STATE_DECISION on ORIENT_DONE; STATE_IDLE on ORIENT_FAIL.
        //   STATE_DECISION - pick layout A or B from the range. Next: STATE_ALIGN
        //                    once the range is in a window; waits otherwise.
        //   STATE_ALIGN    - turn onto leg 1. Next: STATE_PATHFIND when
        //                    rotation_complete.
        //   STATE_PATHFIND - FSM 4: three legs, two turns. Next: STATE_STORAGE
        //                    on PATH_DONE.
        //   STATE_STORAGE  - FSM 2 at 560 mm: face the deposit marker, then set
        //                    the approach d = r - 220 - 150 mm. Next: STATE_DEPOSIT
        //                    on ORIENT_DONE; STATE_IDLE on ORIENT_FAIL.
        //   STATE_DEPOSIT  - drive the approach with heading hold. Next:
        //                    STATE_IDLE when distance_complete, or on a bumper (step 2).
        // Any state goes to STATE_IDLE on wheel drop or B1 (step 1), or on
        // REFLEX_GAVE_UP (step 3).
        switch (current_state) {
            case STATE_IDLE: // motors off, wait for B0
                Kobuki_Drive(0);
                Pathfind_Reset();
                Reflex_Reset();
                orient_step = ORIENT_IDLE;

                if (kobuki.button == BUTTON_0_START) {
                    current_state = STATE_ORIENT;
                }
                break;

            case STATE_ORIENT: { // FSM 2 at 840 mm: face the start bollard
                uint8_t orient_result = Orient_Update(&kobuki, &ultrasonic, &navigation_object, ORIENT_MAX_RANGE_MM);

                if (orient_result == ORIENT_DONE) {
                    current_state = STATE_DECISION;
                } else if (orient_result == ORIENT_FAIL) { // nothing found
                    current_state = STATE_IDLE;
                }
                break;
            }
            case STATE_DECISION: // pick layout A or B; waits if outside both windows
                if (Decision_Determine_Layout(ultrasonic.range_mm)) {
                    Kobuki_Rotate_Angle_Setpoint(kobuki.angle, path_orient_angle);
                    current_state = STATE_ALIGN;
                }
                break;
            case STATE_ALIGN: // turn onto leg 1
                if (!rotation_complete) {
                    Kobuki_Rotate_Angle(kobuki.angle);
                } else {
                    current_state = STATE_PATHFIND;
                }
                break;
            case STATE_PATHFIND: // FSM 4: three legs, two turns
                if (Pathfind_Update(&kobuki) == PATH_DONE) {
                    current_state = STATE_STORAGE;
                }
                break;
            case STATE_STORAGE: { // FSM 2 at 560 mm: face the deposit marker
                uint8_t orient_result = Orient_Update(&kobuki, &ultrasonic, &navigation_object, STORAGE_MAX_RANGE_MM);

                if (orient_result == ORIENT_DONE) {
                    // d = r - 220 - 150 mm
                    int16_t approach = (int16_t)ultrasonic.range_mm - SENSOR_OFFSET_MM - FINISHING_DISTANCE_MM;
                    if (approach < 0) {
                        approach = 0;
                    }
                    Kobuki_Heading_Hold_Set(kobuki.angle);
                    Kobuki_Drive_Distance_Setpoint(kobuki.distance, approach);
                    current_state = STATE_DEPOSIT;
                } else if (orient_result == ORIENT_FAIL) { // nothing found
                    current_state = STATE_IDLE;
                }
                break;
            }
            case STATE_DEPOSIT: // drive the approach with heading hold
                if (!distance_complete) {
                    Kobuki_Drive_Distance_Hold(kobuki.distance, kobuki.angle);
                } else {
                    Kobuki_Drive(0);
                    current_state = STATE_IDLE;
                }
                break;

            default:
                Kobuki_Drive(0);
                current_state = STATE_IDLE;
                break;
        }
    }
}

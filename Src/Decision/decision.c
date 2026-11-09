// ============================================================================
// File:        decision.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Layout decision. Picks layout A or B from the range to the
//              start bollard and loads that layout's path parameters
//              (report Table 5) for FSM 4.
// FSM:         Used by DECISION in FSM 1 (report 4.1).
// Provenance:  Original.
// ============================================================================

#include "decision.h"

// Path parameters per layout (report Table 5). Positive is counter-clockwise.
// Legs and turns come from the geometric analysis (report 3.1, 3.2): drive a
// leg, turn ±90°, drive a second leg, turn ±135° to face the entrance, then
// drive 920 mm into storage. ORIENT_ANGLE is the turn from the scanned
// bollard onto leg 1. Angles in degrees, distances in mm.
#define B_ORIENT_ANGLE_DEG 83
#define A_ORIENT_ANGLE_DEG -95
#define A_ROTATION_1_DEG 90
#define B_ROTATION_1_DEG -90
#define A_ROTATION_2_DEG 135
#define B_ROTATION_2_DEG -135
#define A_DISTANCE_1_MM 1200
#define A_DISTANCE_2_MM 1400
#define B_DISTANCE_1_MM 1400
#define B_DISTANCE_2_MM 1200
#define THIRD_DISTANCE_MM 920

Layout grid_layout;

// 1 if distance (mm) is in the layout A window, 610-840 mm.
static uint8_t in_layout_a(uint16_t distance)
{
    return (distance >= LAYOUT_SPLIT_MM) && (distance <= LAYOUT_A_MAX_MM);
}

// 1 if distance (mm) is in the layout B window, 520-609 mm.
static uint8_t in_layout_b(uint16_t distance)
{
    return (distance >= LAYOUT_B_MIN_MM) && (distance < LAYOUT_SPLIT_MM);
}

// Original.
// Called from DECISION in FSM 1. Returns 1 once a layout's path parameters are loaded.
uint8_t Decision_Determine_Layout(uint16_t distance)
{
    if (in_layout_a(distance)) {
        path_orient_angle = A_ORIENT_ANGLE_DEG;
        path_first_angle = A_ROTATION_1_DEG;
        path_second_angle = A_ROTATION_2_DEG;
        path_first_drive_distance = A_DISTANCE_1_MM;
        path_second_drive_distance = A_DISTANCE_2_MM;
        path_third_drive_distance = THIRD_DISTANCE_MM;
        grid_layout = PATH_A;
        return 1;
    } else if (in_layout_b(distance)) {
        path_orient_angle = B_ORIENT_ANGLE_DEG;
        path_first_angle = B_ROTATION_1_DEG;
        path_second_angle = B_ROTATION_2_DEG;
        path_first_drive_distance = B_DISTANCE_1_MM;
        path_second_drive_distance = B_DISTANCE_2_MM;
        path_third_drive_distance = THIRD_DISTANCE_MM;
        grid_layout = PATH_B;
        return 1;
    }

    return 0;
}

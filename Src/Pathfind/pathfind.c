// ============================================================================
// File:        pathfind.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Path following. Drives three legs with two turns between them,
//              using heading hold on every leg. Turns are aimed from the
//              leg 1 heading theta 0, so turn errors don't build up.
// FSM:         FSM 4 – path following (report 4.4). Run by PATHFIND in FSM 1.
// Provenance:  Original.
// ============================================================================

#include "pathfind.h"
#include "kobuki.h"

PathfindState path_state = PATH_IDLE;
int16_t path_orient_angle = 0;
int16_t path_first_angle = 0;
int16_t path_second_angle = 0;
uint16_t path_first_drive_distance = 0;
uint16_t path_second_drive_distance = 0;
uint16_t path_third_drive_distance = 0;

// Heading at the start of leg 1. Later turns are relative to this.
static int16_t path_ref_angle = 0;

// ref heading + deg, as gyro ticks (1/100 deg, -18000..17999).
// Params: ref_tick (gyro ticks, 0.01 deg), deg (degrees, +ve CCW). Maths: h = ref + deg × 100, wrapped.
static int16_t heading_plus(int16_t ref_tick, int16_t deg)
{
    int32_t h = (int32_t)ref_tick + (int32_t)deg * CENTIDEG_PER_DEG;
    while (h >= HALF_TURN_CENTIDEG) {
        h -= FULL_TURN_CENTIDEG;
    }
    while (h < -HALF_TURN_CENTIDEG) {
        h += FULL_TURN_CENTIDEG;
    }
    return (int16_t)h;
}

// Original.
void Pathfind_Reset(void)
{
    path_state = PATH_IDLE;
}

// Original.
// FSM 4: Path. Drive states can be interrupted by FSM 5, then carry on.
uint8_t Pathfind_Update(Kobuki_Typedef *kobuki)
{
    // FSM 4: path following.
    //   PATH_IDLE     - save theta 0, hold it, load leg 1. Next: FIRST_DRIVE.
    //   FIRST_DRIVE   - leg 1 with heading hold. Next: FIRST_ROTATE when
    //                   distance_complete, aiming at theta 0 + T1.
    //   FIRST_ROTATE  - turn 1. Next: SECOND_DRIVE when rotation_complete,
    //                   holding theta 0 + T1.
    //   SECOND_DRIVE  - leg 2 with heading hold. Next: SECOND_ROTATE when
    //                   distance_complete, aiming at theta 0 + T1 + T2.
    //   SECOND_ROTATE - turn 2. Next: THIRD_DRIVE when rotation_complete,
    //                   holding theta 0 + T1 + T2.
    //   THIRD_DRIVE   - leg 3 into storage. Next: PATH_IDLE, returning
    //                   PATH_DONE, when distance_complete.
    // FSM 5 can pause any drive state; the leg then carries on.
    switch (path_state) {
        case PATH_IDLE: // save the start heading, load leg 1
            path_ref_angle = kobuki->angle;
            Kobuki_Heading_Hold_Set(path_ref_angle);
            Kobuki_Drive_Distance_Setpoint(kobuki->distance, path_first_drive_distance);
            path_state = FIRST_DRIVE;
            break;
        case FIRST_DRIVE: // leg 1
            if (!distance_complete) {
                Kobuki_Drive_Distance_Hold(kobuki->distance, kobuki->angle);
            } else {
                Kobuki_Rotate_Angle_Setpoint(path_ref_angle, path_first_angle);
                path_state = FIRST_ROTATE;
            }
            break;
        case FIRST_ROTATE: // turn 1
            if (!rotation_complete) {
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                Kobuki_Heading_Hold_Set(heading_plus(path_ref_angle, path_first_angle));
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, path_second_drive_distance);
                path_state = SECOND_DRIVE;
            }
            break;
        case SECOND_DRIVE: // leg 2
            if (!distance_complete) {
                Kobuki_Drive_Distance_Hold(kobuki->distance, kobuki->angle);
            } else {
                Kobuki_Rotate_Angle_Setpoint(path_ref_angle, (int16_t)(path_first_angle + path_second_angle));
                path_state = SECOND_ROTATE;
            }
            break;
        case SECOND_ROTATE: // turn 2
            if (!rotation_complete) {
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                Kobuki_Heading_Hold_Set(heading_plus(path_ref_angle, (int16_t)(path_first_angle + path_second_angle)));
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, path_third_drive_distance);
                path_state = THIRD_DRIVE;
            }
            break;
        case THIRD_DRIVE: // leg 3, into storage
            if (!distance_complete) {
                Kobuki_Drive_Distance_Hold(kobuki->distance, kobuki->angle);
            } else {
                path_state = PATH_IDLE;
                return PATH_DONE; // FSM 1 moves to STORAGE
            }
            break;
        default:
            break;
    }

    return PATH_RUNNING;
}

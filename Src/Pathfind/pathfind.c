#include "pathfind.h"
#include "kobuki.h"

pathfind_state path_state = PATH_IDLE;
int16_t path_orient_angle = 0;
int16_t path_first_angle = 0;
int16_t path_second_angle = 0;
uint16_t path_first_drive_distance = 0;
uint16_t path_second_drive_distance = 0;
uint16_t path_third_drive_distance = 0;

// Heading at the start of leg 1. Later turns are relative to this.
static int16_t path_ref_angle = 0;

// ref heading + deg, as gyro ticks (1/100 deg, -18000..17999).
static int16_t heading_plus(int16_t ref_tick, int16_t deg)
{
    int32_t h = (int32_t)ref_tick + (int32_t)deg * 100;
    while (h >= 18000) {
        h -= 36000;
    }
    while (h < -18000) {
        h += 36000;
    }
    return (int16_t)h;
}


void pathfind_reset(void)
{
    path_state = PATH_IDLE;
}

// FSM 4: Path. Drive states can be interrupted by FSM 5, then carry on.
uint8_t pathfind_fsm(Kobuki_Typedef *kobuki)
{
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
                return 1; // FSM 1 moves to STORAGE
            }
            break;
        default:
            break;
    }

    return 0;
}

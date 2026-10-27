#include "pathfind.h"
#include "kobuki.h"

pathfind_state path_state = PATH_IDLE;
int16_t path_orient_angle = 0;
int16_t path_first_angle = 0;
int16_t path_second_angle = 0;
uint16_t path_first_drive_distance = 0;
uint16_t path_second_drive_distance = 0;
uint16_t path_third_drive_distance = 0;

/* Heading at the start of leg 1. Every path turn is aimed relative to this,
   so a bump/cliff nudge (or turn error) on one leg does not carry into the next. */
static int16_t path_ref_angle = 0;


void pathfind_reset(void)
{
    path_state = PATH_IDLE;
}

uint8_t pathfind_fsm(Kobuki_Typedef *kobuki)
{
    switch (path_state) {
        case PATH_IDLE:
            path_ref_angle = kobuki->angle;
            Kobuki_Drive_Distance_Setpoint(kobuki->distance, path_first_drive_distance);
            path_state = FIRST_DRIVE;
            break;
        case FIRST_DRIVE:
            if (!distance_complete) {
                Kobuki_Drive_Distance(kobuki->distance);
            } else {
                Kobuki_Rotate_Angle_Setpoint(path_ref_angle, path_first_angle);
                path_state = FIRST_ROTATE;
            }
            break;
        case FIRST_ROTATE:
            if (!rotation_complete) {
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, path_second_drive_distance);
                path_state = SECOND_DRIVE;
            }
            break;
        case SECOND_DRIVE:
            if (!distance_complete) {
                Kobuki_Drive_Distance(kobuki->distance);
            } else {
                Kobuki_Rotate_Angle_Setpoint(path_ref_angle, (int16_t)(path_first_angle + path_second_angle));
                path_state = SECOND_ROTATE;
            }
            break;
        case SECOND_ROTATE:
            if (!rotation_complete) {
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, path_third_drive_distance);
                path_state = THIRD_DRIVE;
            }
            break;
        case THIRD_DRIVE:
            if (!distance_complete) {
                Kobuki_Drive_Distance(kobuki->distance);
            } else {
                path_state = PATH_IDLE;
                return 1;
            }
            break;
        default:
            break;
    }

    return 0;
}

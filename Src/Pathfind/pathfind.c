#include "pathfind.h"

pathfind_state path_state = PATH_IDLE;
int16_t path_angle = 0;
uint16_t path_first_drive_distance = 0;
uint16_t path_second_drive_distance = 0;

void pathfind_reset(void)
{
    path_state = PATH_IDLE;
}

uint8_t pathfind_fsm(Kobuki_Typedef *kobuki)
{
    switch (path_state) {
        case PATH_IDLE:
            Kobuki_Drive_Distance_Setpoint(kobuki->distance, path_first_drive_distance);
            path_state = FIRST_DRIVE;
            break;
        case FIRST_DRIVE:
            if (!distance_complete) {
                Kobuki_Drive_Distance(kobuki->distance);
            } else {
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, path_angle);
                path_state = MID_ROTATE;
            }
            break;
        case MID_ROTATE:
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
                // will work out what angle to rotate to here.
                path_state = PATH_IDLE;
                return 1;
            }
            break;
        default:
            break;
    }

    return 0;
}

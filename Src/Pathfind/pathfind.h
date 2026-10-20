#pragma once

#include "kobuki.h"

extern int16_t path_orient_angle;
extern int16_t path_first_angle;
extern int16_t path_second_angle;
extern uint16_t path_first_drive_distance;
extern uint16_t path_second_drive_distance;
extern uint16_t path_third_drive_distance;


 typedef enum {
    PATH_IDLE,
    FIRST_DRIVE,
    FIRST_ROTATE,
    SECOND_DRIVE,
    SECOND_ROTATE,
    THIRD_DRIVE
} pathfind_state;

void pathfind_reset(void);
uint8_t pathfind_fsm(Kobuki_Typedef *kobuki);
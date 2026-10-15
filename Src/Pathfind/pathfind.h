#pragma once

#include "kobuki.h"

extern int16_t path_angle;
extern uint16_t path_first_drive_distance;
extern uint16_t path_second_drive_distance;

 typedef enum {
    PATH_IDLE,
    FIRST_DRIVE,
    MID_ROTATE,
    SECOND_DRIVE
} pathfind_state;

void pathfind_reset(void);
uint8_t pathfind_fsm(Kobuki_Typedef *kobuki);
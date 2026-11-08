#pragma once

#include "kobuki.h"

// Path parameters (report Table 4). Loaded by Determine_Layout in DECISION.
extern int16_t path_orient_angle;
extern int16_t path_first_angle;
extern int16_t path_second_angle;
extern uint16_t path_first_drive_distance;
extern uint16_t path_second_drive_distance;
extern uint16_t path_third_drive_distance;


// FSM 4: Path (report 4.4). Run by PATHFIND in FSM 1.
typedef enum {
    PATH_IDLE,
    FIRST_DRIVE,
    FIRST_ROTATE,
    SECOND_DRIVE,
    SECOND_ROTATE,
    THIRD_DRIVE
} pathfind_state;

extern pathfind_state path_state;

void pathfind_reset(void);
uint8_t pathfind_fsm(Kobuki_Typedef *kobuki);
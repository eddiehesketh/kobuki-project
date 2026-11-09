// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        pathfind.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Path following. Drives three legs with two turns between them,
//              using heading hold on every leg. Turns are aimed from the
//              leg 1 heading theta 0, so turn errors don't build up.
// FSM:         FSM 4 – path following (report 4.4). Run by PATHFIND in FSM 1.
// Provenance:  Original.
// ============================================================================

#pragma once

#include "kobuki.h"

// Pathfind_Update return values.
#define PATH_RUNNING 0U // a leg or turn is in progress
#define PATH_DONE 1U // leg 3 finished

// Path parameters (report Table 5). Loaded by Decision_Determine_Layout in DECISION.
extern int16_t path_orient_angle; // ALIGN turn onto leg 1 (degrees)
extern int16_t path_first_angle; // turn 1 from theta 0 (degrees)
extern int16_t path_second_angle; // turn 2, added to turn 1 (degrees)
extern uint16_t path_first_drive_distance; // leg 1 (mm)
extern uint16_t path_second_drive_distance; // leg 2 (mm)
extern uint16_t path_third_drive_distance; // leg 3 (mm)

// FSM 4: Path (report 4.4). Run by PATHFIND in FSM 1.
typedef enum {
    PATH_IDLE,
    FIRST_DRIVE,
    FIRST_ROTATE,
    SECOND_DRIVE,
    SECOND_ROTATE,
    THIRD_DRIVE
} PathfindState;

extern PathfindState path_state;

// Pathfind_Reset
// Provenance: Original.
// Purpose:    Return FSM 4 to PATH_IDLE.
// Params:     none.
// Returns:    nothing.
// Effects:    sets path_state; no motor command.
void Pathfind_Reset(void);

// Pathfind_Update
// Provenance: Original.
// Purpose:    One step of FSM 4.
// Params:     kobuki - current robot state.
// Returns:    PATH_DONE once leg 3 finishes, else PATH_RUNNING.
// Effects:    sends drive and rotate commands; sets the heading hold and the
//             distance and rotation setpoints.
// Maths:      leg headings θ0, θ0 + T1, θ0 + T1 + T2.
uint8_t Pathfind_Update(Kobuki_Typedef *kobuki);

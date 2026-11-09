// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        decision.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Layout decision. Picks layout A or B from the range to the
//              start bollard and loads that layout's path parameters
//              (report Table 5) for FSM 4.
// FSM:         Used by DECISION in FSM 1 (report 4.1).
// Provenance:  Original.
// ============================================================================

#pragma once

#include <stdint.h>
#include "pathfind.h"

// Sensor face sits this far behind the robot edge. Diagram gaps are to the edge.
// mm, measured on the robot.
#define SENSOR_OFFSET_MM 220

// Layout windows, mm from the sensor face.
// B: LAYOUT_B_MIN_MM .. LAYOUT_SPLIT_MM - 1
// A: LAYOUT_SPLIT_MM .. LAYOUT_A_MAX_MM
// LAYOUT_A_MAX_MM is also the orient scan limit, so farther bollards are ignored.
// A1 and B1 sit at different distances from the start circle; the windows
// were settled by trial and error on the course (report 3.4).
#define LAYOUT_B_MIN_MM 520
#define LAYOUT_SPLIT_MM 610
#define LAYOUT_A_MAX_MM 840

typedef enum {
    PATH_A,
    PATH_B
} Layout;

extern Layout grid_layout;

// Decision_Determine_Layout
// Provenance: Original.
// Purpose:    Pick the layout from the range to the start bollard and load its
//             path parameters. Called from DECISION in FSM 1.
// Params:     distance - ultrasonic range (mm) while facing the start bollard.
// Returns:    1 once a layout's path parameters are loaded; 0 if the range is
//             outside both windows (FSM 1 waits).
// Effects:    writes the path_* parameters and grid_layout.
uint8_t Decision_Determine_Layout(uint16_t distance);

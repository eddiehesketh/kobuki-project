#pragma once
#include "kobuki.h"
#include <stdint.h>


// FSM 3: Edge scan (report 4.3). Run by ORIENT_SCANNING in FSM 2.
typedef enum {
    CLEAR_OBJECT = 0,
    FIND_ANGLE_1,
    FIND_ANGLE_2,
    SCAN_COMPLETE,
    SCAN_FAILED
} NavigationScanState;

typedef struct {
    NavigationScanState state;

    int16_t angle1; // theta 1: object enters the beam
    int16_t angle2; // theta 2: object leaves the beam
    int16_t midpoint; // theta mid: heading to face

    int16_t previous_angle;
    int32_t swept_angle;

    // An edge counts after SCAN_DEBOUNCE_SAMPLES readings. The saved angle is the first.
    uint8_t run_count;
    int16_t run_start_angle;
} NavigationScan;


int16_t Navigation_Midpoint(int16_t first_angle, int16_t second_angle);
void Navigation_Scan_Start(NavigationScan *scan, int16_t current_angle, uint8_t detected);
void Navigation_Scan_Update(NavigationScan *scan, int16_t current_angle, uint8_t detected);
void Navigation_Orient_Start(int16_t target_angle); // turn to the bollard midpoint
uint8_t Navigation_Orient_Update(const Kobuki_Typedef *kobuki); // 1 when the turn finishes

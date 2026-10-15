#pragma once
#include "kobuki.h"
#include <stdint.h>


typedef enum {
    CLEAR_OBJECT = 0,
    FIND_ANGLE_1,
    FIND_ANGLE_2,
    SCAN_COMPLETE,
    SCAN_FAILED
} NavigationScanState;

typedef struct {
    NavigationScanState state;

    int16_t angle1;
    int16_t angle2;
    int16_t midpoint;

    int16_t previous_angle;
    int32_t swept_angle;
} NavigationScan;


// Calculate the midpoint between two angles.
int16_t Navigation_Midpoint(int16_t first_angle, int16_t second_angle);
// Start a new navigation scan.
void Navigation_Scan_Start(NavigationScan *scan, int16_t current_angle, uint8_t detected);
/// Update the current navigation scan.
void Navigation_Scan_Update(NavigationScan *scan, int16_t current_angle, uint8_t detected);
// Return back to the midpt of the bollard
void Navigation_Orient_Start(int16_t target_angle);
// Update the current orientation.
uint8_t Navigation_Orient_Update(const Kobuki_Typedef *kobuki);

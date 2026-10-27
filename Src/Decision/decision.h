#pragma once

#include <stdint.h>
#include "pathfind.h"

/* Sensor face sits this far behind the robot edge. Diagram gaps are to the edge. */
#define SENSOR_DISTANCE 220

/* Layout windows on the ultrasonic reading (mm from the sensor face):
     B: LAYOUT_B_MIN_MM .. LAYOUT_SPLIT_MM - 1
     A: LAYOUT_SPLIT_MM .. LAYOUT_A_MAX_MM
   LAYOUT_A_MAX_MM is also the ORIENT scan's max range, so nothing further
   away (e.g. the bollards inside the enclosure) is ever scanned. */
#define LAYOUT_B_MIN_MM 520
#define LAYOUT_SPLIT_MM 610
#define LAYOUT_A_MAX_MM 820

typedef enum {
    PATH_A,
    PATH_B
} Layout;

extern Layout grid_layout;

uint8_t Determine_Layout(uint16_t distance);


#pragma once

#include <stdint.h>
#include "pathfind.h"

/* Sensor face sits this far behind the robot edge. Diagram gaps are to the edge. */
#define SENSOR_DISTANCE 220

typedef enum {
    PATH_A,
    PATH_B
} Layout;

extern Layout grid_layout;

uint8_t Determine_Layout(uint16_t distance);


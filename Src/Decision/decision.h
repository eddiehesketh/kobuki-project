#pragma once

#include <stdint.h>
#include "pathfind.h"

typedef enum {
    PATH_A,
    PATH_B
} Layout;

extern Layout grid_layout;

uint8_t Determine_Layout(uint16_t distance);


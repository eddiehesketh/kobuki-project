#pragma once

#include "navigation.h"
#include "ultrasonic.h"

typedef enum {
    ORIENT_IDLE,
    ORIENT_SCANNING,
    ORIENT_TURNING
} OrientStep;

extern OrientStep orient_step;

/* 0 while orienting, 1 when the turn finishes, 2 when the scan fails. */
uint8_t orient_fsm(Kobuki_Typedef *kobuki, US_Typedef *ultrasonic,
                   NavigationScan *navigation_object, uint16_t max_range_mm);

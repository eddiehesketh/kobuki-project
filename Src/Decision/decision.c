#include "decision.h"

#define B_ORIENT_ANGLE 85
#define A_ORIENT_ANGLE -95
#define A_ROTATION_1 90
#define B_ROTATION_1 -90 // subject to change
#define A_ROTATION_2 135  
#define B_ROTATION_2 -135 
#define A_DISTANCE_1 1200
#define A_DISTANCE_2 1400
#define B_DISTANCE_1 1400
#define B_DISTANCE_2 1200
#define THIRD_DISTANCE 920
#define BUFFER 100
#define A_BUFFER 35

Layout grid_layout;


#define BOLLARD_A1_DISTANCE (375 + A_BUFFER)
#define BOLLARD_B1_DISTANCE (225 + BUFFER)
#define NOISE 100

/* Expected sensor readings: A = 595 mm, B = 445 mm */
#define EXPECTED_A (BOLLARD_A1_DISTANCE + SENSOR_DISTANCE)
// #define EXPECTED_B (BOLLARD_B1_DISTANCE + SENSOR_DISTANCE)
#define EXPECTED_B (620)

#define LAYOUT_SPLIT ((EXPECTED_A + EXPECTED_B) / 2)   /* 520 mm */

static uint8_t LayoutA(uint16_t distance) {
    /* 520 to 695 mm */
    return (distance >= LAYOUT_SPLIT) && (distance <= EXPECTED_A + NOISE);
}

static uint8_t LayoutB(uint16_t distance) {
    /* 345 to 519 mm */
    return (distance < LAYOUT_SPLIT) && (distance >= EXPECTED_B - NOISE);
}


uint8_t Determine_Layout(uint16_t distance) {
    if (LayoutA(distance)) {
        path_orient_angle = A_ORIENT_ANGLE;
        path_first_angle = A_ROTATION_1;
        path_second_angle = A_ROTATION_2;
        path_first_drive_distance = A_DISTANCE_1;
        path_second_drive_distance = A_DISTANCE_2;
        path_third_drive_distance = THIRD_DISTANCE;
        grid_layout = PATH_A;
        return 1;
    } else if (LayoutB(distance)) {
        path_orient_angle = B_ORIENT_ANGLE;
        path_first_angle = B_ROTATION_1;
        path_second_angle = B_ROTATION_2;
        path_first_drive_distance = B_DISTANCE_1;
        path_second_drive_distance = B_DISTANCE_2;
        path_third_drive_distance = THIRD_DISTANCE;
        grid_layout = PATH_B;
        return 1;
    }

    return 0;
}


#include "decision.h"

#define BOLLARD_A1_DISTANCE 375
#define BOLLARD_B1_DISTANCE 225
#define NOISE 50 

#define A_ROTATION 90
#define B_ROTATION -90
#define A_DISTANCE_1 10000 // yet to be determined
#define A_DISTANCE_2 10000 // yet to be determined
#define B_DISTANCE_1 10000 // yet to be determined
#define B_DISTANCE_2 10000 // yet to be determined

Layout grid_layout;



static uint8_t LayoutA(uint16_t distance) {
    if (distance <= BOLLARD_A1_DISTANCE + NOISE && distance >= BOLLARD_A1_DISTANCE - NOISE) {
        return 1;
    }

    return 0;
}
static uint8_t LayoutB(uint16_t distance) {
    if (distance <= BOLLARD_B1_DISTANCE + NOISE && distance >= BOLLARD_B1_DISTANCE - NOISE) {
        return 1;
    }

    return 0;
}

uint8_t Determine_Layout(uint16_t distance) {
    if (LayoutA(distance)) {
        path_angle = A_ROTATION;
        path_first_drive_distance = A_DISTANCE_1;
        path_second_drive_distance = A_DISTANCE_2;
        grid_layout = PATH_A;
        return 1;
    } else if (LayoutB(distance)) {
        path_angle = B_ROTATION;
        path_first_drive_distance = B_DISTANCE_1;
        path_second_drive_distance = B_DISTANCE_2;
        grid_layout = PATH_B;
        return 1;
    }

    return 0;
}


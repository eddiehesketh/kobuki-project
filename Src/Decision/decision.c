#include "decision.h"

#define BOLLARD_A1_DISTANCE 375
#define BOLLARD_B1_DISTANCE 225
#define NOISE 50 


uint8_t LayoutA(uint16_t distance) {
    if (distance <= BOLLARD_A1_DISTANCE + NOISE && distance >= BOLLARD_A1_DISTANCE - NOISE) {
        return 1;
    }

    return 0;
}
uint8_t LayoutB(uint16_t distance) {
    if (distance <= BOLLARD_B1_DISTANCE + NOISE && distance >= BOLLARD_B1_DISTANCE - NOISE) {
        return 1;
    }

    return 0;
}

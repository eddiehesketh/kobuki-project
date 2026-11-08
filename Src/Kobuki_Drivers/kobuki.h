#pragma once

#include <stdint.h>


extern uint8_t rotation_complete;
extern uint8_t distance_complete;

typedef struct {
    uint8_t cliff;
    uint8_t bumper;
    uint8_t wheeldrop;
    uint8_t button;
    uint16_t distance; // raw wheel encoder count
    int16_t angle; // gyro, hundredths of a degree
    uint8_t rotation_complete; // 1 when the turn finishes
    uint8_t distance_complete; // 1 when the drive finishes
} Kobuki_Typedef;

typedef enum {
    OFF = 0,
    RED,
    GREEN,
    YELLOW
} Kobuki_LED_Enum;

void Kobuki_Update_LEDs(Kobuki_LED_Enum led_1_colour, Kobuki_LED_Enum led_2_colour);

#define SPEED_LIMIT (int16_t)(150)

uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback);
void Kobuki_Tx(uint8_t *payload, uint8_t size_payload);
void Kobuki_Read(Kobuki_Typedef *kobuki);
void Kobuki_Drive(int16_t speed); // +ve is forward
void Kobuki_Rotate(int16_t speed); // +ve is CCW
void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg);
void Kobuki_Rotate_Angle(int16_t current_angle_tick);
void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance);
void Kobuki_Drive_Distance(uint16_t current_distance_tick);
int16_t Kobuki_Drive_Distance_Remaining(uint16_t current_distance_tick); // mm left
void Kobuki_Drive_Arc(int16_t speed, int16_t radius); // +ve radius curves left, 0 is straight
void Kobuki_Heading_Hold_Set(int16_t angle_tick); // gyro ticks, 1/100 deg
void Kobuki_Drive_Distance_Hold(uint16_t current_distance_tick, int16_t current_angle_tick);

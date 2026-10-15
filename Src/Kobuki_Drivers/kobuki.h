#pragma once

#include <stdint.h>


extern uint8_t rotation_complete;
extern uint8_t distance_complete;

//Kobuki structure
typedef struct {
    uint8_t cliff;
    uint8_t bumper;
    uint8_t wheeldrop;
    uint8_t button;

    // add the movement feedback fields from the project resource //

    uint16_t distance;         // Raw wheel encoder count
    int16_t angle;             // Gyro angle in hundredths of a degree

    uint8_t rotation_complete; // 1 when the requested turn finishes
    uint8_t distance_complete; // 1 when the requested drive finishes
} Kobuki_Typedef;

typedef enum {
    OFF = 0,
    RED,
    GREEN,
    YELLOW
} Kobuki_LED_Enum;

void Kobuki_Update_LEDs(Kobuki_LED_Enum led_1_colour, Kobuki_LED_Enum led_2_colour);

#define SPEED_LIMIT (int16_t)(150)

// Receive payload
uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback);
// Transmit payload
void Kobuki_Tx(uint8_t *payload, uint8_t size_payload);
// Kobuki receive payload and store
void Kobuki_Read(Kobuki_Typedef *kobuki);
// Kobuki drive forward/backward (+ve is forward)
void Kobuki_Drive(int16_t speed);
// Kobuki rotate CW/CCW (+ve is CCW)
void Kobuki_Rotate(int16_t speed);
// Kobuki rotate to a specific angle
void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg);
// Kobuki rotate to a specific angle
void Kobuki_Rotate_Angle(int16_t current_angle_tick);
// Kobuki drive to a specific distance
void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance);
// Kobuki drive to a specific distance
void Kobuki_Drive_Distance(uint16_t current_distance_tick);
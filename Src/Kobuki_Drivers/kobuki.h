// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        kobuki.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Kobuki serial protocol and motion control. Reads the feedback
//              packet, sends drive, rotate and arc commands, and runs the
//              rotation, distance and heading-hold controllers.
// FSM:         Motion layer used by FSM 1, FSM 2, FSM 4 and FSM 5.
// Provenance:  Mixed; see each function below.
//              SPEED_LIMIT, Kobuki_LED_Enum and the distance, angle,
//              rotation_complete and distance_complete fields:
//              Provided – project resource (Microcontroller Project Resources, Dr M. Forbes).
// ============================================================================

#pragma once

#include <stdint.h>

// Original. Gyro angles are in centidegrees (gyro ticks, 0.01 deg).
#define CENTIDEG_PER_DEG 100
#define HALF_TURN_CENTIDEG 18000
#define FULL_TURN_CENTIDEG 36000

extern uint8_t rotation_complete;
extern uint8_t distance_complete;

typedef struct {
    uint8_t cliff; // bit 0 right, bit 1 centre, bit 2 left
    uint8_t bumper; // bit 0 right, bit 1 centre, bit 2 left
    uint8_t wheeldrop; // non-zero while a wheel is lifted
    uint8_t button; // bit 0 B0, bit 1 B1, bit 2 B2
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

// Kobuki_Update_LEDs
// Provenance: Provided – project resource (Microcontroller Project Resources, Dr M. Forbes).
// Purpose:    Set both LED colours and keep the external power outputs on.
// Params:     led_1_colour, led_2_colour - colour for LED 1 and LED 2.
// Returns:    nothing.
// Effects:    sends a general-purpose output command.
void Kobuki_Update_LEDs(Kobuki_LED_Enum led_1_colour, Kobuki_LED_Enum led_2_colour);

// Provided – project resource. Wheel speed limit (mm/s).
#define SPEED_LIMIT (int16_t)(150)

// Kobuki_Rx
// Provenance: Provided – practical drivers; modified: drains extra payload
//             bytes, flags a short payload.
// Purpose:    Read one feedback packet.
// Params:     feedback      - buffer for the first size_feedback payload bytes.
//             size_feedback - number of bytes wanted.
// Returns:    0 if the packet is good; 1 on a bad checksum or short payload.
// Effects:    blocks until a full packet arrives (the loop's 50 Hz pacing).
uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback);

// Kobuki_Tx
// Provenance: Provided – practical drivers.
// Purpose:    Send one command packet: 0xAA 0x55, length, payload, XOR checksum.
// Params:     payload      - command bytes.
//             size_payload - number of payload bytes.
// Returns:    nothing.
// Effects:    writes to the UART; blocks until every byte is sent.
void Kobuki_Tx(uint8_t *payload, uint8_t size_payload);

// Kobuki_Read
// Provenance: Adapted from project resource; single cast/shift byte unpacking.
// Purpose:    Read one packet and update kobuki.
// Params:     kobuki - robot state to update.
// Returns:    nothing.
// Effects:    blocks as Kobuki_Rx. Sensor fields keep their old values on a
//             bad packet. Always copies rotation_complete and distance_complete.
void Kobuki_Read(Kobuki_Typedef *kobuki);

// Kobuki_Drive
// Provenance: Provided – practical drivers.
// Purpose:    Drive straight.
// Params:     speed - wheel speed (mm/s), clamped to ±SPEED_LIMIT.
// Returns:    nothing.
// Effects:    sends a drive command.
void Kobuki_Drive(int16_t speed); // +ve is forward

// Kobuki_Rotate
// Provenance: Provided – practical drivers.
// Purpose:    Spin on the spot.
// Params:     speed - wheel speed (mm/s), clamped to ±SPEED_LIMIT.
// Returns:    nothing.
// Effects:    sends a drive command with radius 1 (spin).
void Kobuki_Rotate(int16_t speed); // +ve is CCW

// Kobuki_Rotate_Angle_Setpoint
// Provenance: Provided – project resource (Microcontroller Project Resources, Dr M. Forbes).
// Purpose:    Set a rotation target relative to a reference heading.
// Params:     current_angle_tick - reference heading (gyro ticks, 0.01 deg).
//             rot_angle_deg      - turn from the reference (degrees), +ve CCW.
// Returns:    nothing.
// Effects:    clears rotation_complete; stores the target heading.
void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg);

// Kobuki_Rotate_Angle
// Provenance: Adapted from project resource; plain ±180° short-way wrap
//             instead of the modulo-180 special case; stops the motor on completion.
// Purpose:    One step of the rotation P-controller toward the setpoint.
// Params:     current_angle_tick - gyro heading (gyro ticks, 0.01 deg).
// Returns:    nothing.
// Effects:    sends a rotate command. At zero whole-degree error, stops and
//             sets rotation_complete.
void Kobuki_Rotate_Angle(int16_t current_angle_tick);

// Kobuki_Drive_Distance_Setpoint
// Provenance: Provided – project resource (Microcontroller Project Resources, Dr M. Forbes).
// Purpose:    Set a straight-line distance target from the current encoder count.
// Params:     current_distance_tick - encoder count (ticks).
//             distance              - distance to drive (mm), -ve reverses.
// Returns:    nothing.
// Effects:    clears distance_complete; stores the target tick.
void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance);

// Kobuki_Drive_Distance
// Provenance: Adapted from project resource; signed 16-bit wrap instead of
//             setpoint_alias; stops the motor on completion.
// Purpose:    One step of the distance P-controller, driving straight.
// Params:     current_distance_tick - encoder count (ticks).
// Returns:    nothing.
// Effects:    sends a drive command. Within 2 mm, stops and sets distance_complete.
void Kobuki_Drive_Distance(uint16_t current_distance_tick);

// Kobuki_Drive_Distance_Remaining
// Provenance: Original.
// Purpose:    Distance still to go on the current distance setpoint.
// Params:     current_distance_tick - encoder count (ticks).
// Returns:    remaining distance (mm), -ve if past the target.
// Effects:    none.
// Maths:      d = wrap16(target - current) × MM_PER_TICK.
int16_t Kobuki_Drive_Distance_Remaining(uint16_t current_distance_tick); // mm left

// Kobuki_Drive_Arc
// Provenance: Original.
// Purpose:    Drive along an arc.
// Params:     speed  - speed (mm/s).
//             radius - arc radius (mm).
// Returns:    nothing.
// Effects:    sends a drive command, speed clamped to ±SPEED_LIMIT.
// Maths:      R = v/ω; wheel speed scaled by (R ± b/2)/R, b = 230 mm.
void Kobuki_Drive_Arc(int16_t speed, int16_t radius); // +ve radius curves left, 0 is straight

// Kobuki_Heading_Hold_Set
// Provenance: Original.
// Purpose:    Set the heading Kobuki_Drive_Distance_Hold steers toward.
// Params:     angle_tick - heading to hold (gyro ticks, 0.01 deg).
// Returns:    nothing.
// Effects:    stores the hold heading.
void Kobuki_Heading_Hold_Set(int16_t angle_tick); // gyro ticks, 1/100 deg

// Kobuki_Drive_Distance_Hold
// Provenance: Original.
// Purpose:    One step of the distance P-controller, steering to hold the set heading.
// Params:     current_distance_tick - encoder count (ticks).
//             current_angle_tick    - gyro heading (gyro ticks, 0.01 deg).
// Returns:    nothing.
// Effects:    sends an arc command. Within 2 mm, stops and sets distance_complete.
// Maths:      ω = 0.05 × e_θ (deg) rad/s, |ω| ≤ 0.3; R = v/ω.
void Kobuki_Drive_Distance_Hold(uint16_t current_distance_tick, int16_t current_angle_tick);

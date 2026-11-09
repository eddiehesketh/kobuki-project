// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        kobuki.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Kobuki serial protocol and motion control. Packs and unpacks
//              packets over the UART and runs the rotation, distance and
//              heading-hold controllers once per loop.
// FSM:         Motion layer used by FSM 1, FSM 2, FSM 4 and FSM 5.
// Provenance:  Mixed; each function is labelled below and in kobuki.h.
//              Provided - project resource (Microcontroller Project Resources,
//              Dr M. Forbes): Kobuki_Rotate_Angle_Setpoint,
//              Kobuki_Drive_Distance_Setpoint, Kobuki_Update_LEDs and the
//              macros marked below.
//              Adapted from project resource: Kobuki_Read, Kobuki_Rotate_Angle,
//              Kobuki_Drive_Distance.
//              Provided - practical drivers: Kobuki_Tx, Kobuki_Rx,
//              Kobuki_Drive, Kobuki_Rotate.
//              Original: Kobuki_Drive_Arc, Kobuki_Heading_Hold_Set,
//              Kobuki_Drive_Distance_Hold, Kobuki_Drive_Distance_Remaining.
// ============================================================================

#include <stdint.h>
#include <stdlib.h> // abs()
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"

// ---- Provided - project resource (Microcontroller Project Resources, Dr M. Forbes) ----

#define SIZE_FEEDBACK 26 // feedback payload bytes read each packet

// Packet framing and the base-control command (Kobuki protocol).
#define KOBUKI_HEADER_0 0xAAU // first header byte
#define KOBUKI_HEADER_1 0x55U // second header byte
#define KOBUKI_CMD_BASE_CONTROL 0x01U // base-control command ID
#define KOBUKI_BASE_CONTROL_SIZE 0x04U // speed (2 bytes) + radius (2 bytes)
#define KOBUKI_CMD_GPO 0x0CU // general-purpose output command ID
#define KOBUKI_GPO_SIZE 0x02U // two data bytes

// Feedback packet field offsets.
#define BUMPER 4
#define WHEEL_DROP 5
#define CLIFF 6
#define DIST_LSB 7
#define DIST_MSB 8
#define BUTTON 13
#define ANGLE_LSB 24
#define ANGLE_MSB 25

#define DEG_PER_TICK (1.0 / 100.0) // degrees per gyro tick
#define GAIN_ROTATE (1.0 / 0.90) // Kp for rotation P-controller (mm/s per degree)
#define MIN_SPEED ((int16_t)50) // mm/s, overcomes wheel friction near the target

#define sign(x) ((int8_t)((x) >= 0 ? 1 : -1)) // +1 or -1

#define PI (3.14159)
#define WHEEL_RADIUS (35.0) // mm
#define REVOLUTIONS_PER_TICK ((132.0 / 6545.0) / 52.0) // wheel revolutions per encoder tick

#define MM_PER_TICK (2.0 * PI * WHEEL_RADIUS * REVOLUTIONS_PER_TICK) // mm per encoder tick

#define GAIN_DRIVE (100.0 / 100.0) // Kp for distance P-controller (mm/s per mm), 1.0 is the default

// Whole-degree headings used by the rotation controller.
#define DEG_FULL_TURN 360
#define DEG_HALF_TURN 180

static volatile int32_t setpoint_distance_tick; // distance target (encoder ticks)
static volatile int16_t setpoint_angle_deg; // rotation target (degrees, 0-359)
static volatile int16_t hold_angle_tick; // heading-hold target (gyro ticks, 0.01 deg)

// ---- Original ----

// Turn rate (rad/s) per degree of heading error. 0 disables hold.
// Small, so the robot follows a gentle arc back to the leg heading (report eq. 10, eq. 32).
#define HOLD_GAIN_RAD_S_PER_DEG (0.05)
// Turn-rate cap (rad/s), so a large error still gives a gentle arc (report eq. 32).
#define HOLD_MAX_RATE_RAD_S (0.30)
// Kobuki wheelbase b (mm), from the Kobuki spec.
#define WHEELBASE_MM (230.0)
// A drive counts as finished within this distance of the target (mm).
#define DISTANCE_TOLERANCE_MM 2
// Above this radius (mm) the heading error is tiny, so drive straight.
#define HOLD_STRAIGHT_RADIUS_MM (30000.0)
// Kobuki radius codes: |R| <= 1 means spin on the spot, so arcs use at least 2 mm.
#define ARC_SPIN_RADIUS_MM 1
#define ARC_MIN_RADIUS_MM 2
// The 16-bit encoder count wraps between 65535 and 0.
#define ENCODER_WRAP_TICKS 65536

uint8_t rotation_complete = 0;
uint8_t distance_complete = 0;

// Provided - practical drivers.
void Kobuki_Tx(uint8_t *payload, uint8_t size_payload)
{
    uint8_t checksum = 0, i;

    UART_Tx(KOBUKI_HEADER_0); // header
    UART_Tx(KOBUKI_HEADER_1);
    UART_Tx(size_payload);
    checksum ^= size_payload;

    for (i = 0; i < size_payload; ++i) {
        UART_Tx(payload[i]);
        checksum ^= payload[i];
    }

    UART_Tx(checksum);
}

// Provided - practical drivers; modified: drains extra payload bytes, flags a short payload.
uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback)
{
    uint8_t feedback_header = 0, size_payload = 0, checksum = 0, rx_checksum = 0;
    uint32_t i;

    // Find the 0xAA 0x55 header.
    // No timeout: the Kobuki streams feedback every 20 ms, so this wait is the
    // main loop's 50 Hz pacing. With the Kobuki unplugged, the board waits here.
    while (1) {
        feedback_header = UART_Rx();
        if (feedback_header == KOBUKI_HEADER_0) {
            feedback_header = UART_Rx();
            if (feedback_header == KOBUKI_HEADER_1) {
                break;
            }
        }
    }

    size_payload = UART_Rx();
    checksum ^= size_payload;

    uint32_t limit = (size_payload < size_feedback) ? size_payload : size_feedback;

    for (i = 0; i < limit; i++) {
        feedback[i] = UART_Rx();
        checksum ^= feedback[i];
    }

    // Drain the rest so the checksum still covers the full payload.
    for (i = limit; i < size_payload; i++) {
        checksum ^= UART_Rx();
    }

    rx_checksum = UART_Rx();
    // 1 on a bad checksum or a payload shorter than required.
    return ((checksum != rx_checksum) || (size_payload < size_feedback));
}

// Adapted from project resource: single cast/shift byte unpacking.
void Kobuki_Read(Kobuki_Typedef *kobuki)
{
    uint8_t feedback[SIZE_FEEDBACK];
    uint8_t checksum_result;

    checksum_result = Kobuki_Rx(feedback, SIZE_FEEDBACK);

    if (checksum_result == 0U) {
        kobuki->bumper = feedback[BUMPER];
        kobuki->wheeldrop = feedback[WHEEL_DROP];
        kobuki->cliff = feedback[CLIFF];
        kobuki->button = feedback[BUTTON];

        // Two bytes into a 16-bit encoder count.
        kobuki->distance =
            (uint16_t)feedback[DIST_LSB] |
            ((uint16_t)feedback[DIST_MSB] << 8);

        // Two bytes into a signed gyro angle.
        kobuki->angle = (int16_t)(
            (uint16_t)feedback[ANGLE_LSB] |
            ((uint16_t)feedback[ANGLE_MSB] << 8)
        );
    }

    kobuki->rotation_complete = rotation_complete;
    kobuki->distance_complete = distance_complete;
}

// Provided - practical drivers.
void Kobuki_Drive(int16_t speed)
{
    uint8_t payload[] = {KOBUKI_CMD_BASE_CONTROL, KOBUKI_BASE_CONTROL_SIZE, 0x00, 0x00, 0x00, 0x00};

    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    } else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

// Provided - practical drivers.
void Kobuki_Rotate(int16_t speed)
{
    uint8_t payload[] = {KOBUKI_CMD_BASE_CONTROL, KOBUKI_BASE_CONTROL_SIZE, 0x00, 0x00, ARC_SPIN_RADIUS_MM, 0x00}; // radius 1: spin

    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    } else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

// Provided - project resource (Microcontroller Project Resources, Dr M. Forbes).
void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg)
{
    volatile int32_t init_angle;

    rotation_complete = 0;

    // Gyro ticks to a heading in the 0-360 degree range.
    init_angle = (int32_t)(
        (double)(current_angle_tick + (int32_t)HALF_TURN_CENTIDEG)
        * DEG_PER_TICK
    );

    // Target heading after the requested rotation.
    setpoint_angle_deg =
        (init_angle + rot_angle_deg + DEG_FULL_TURN) % DEG_FULL_TURN;
}

// Adapted from project resource: plain ±180° short-way wrap instead of the modulo-180 special case; stops the motor on completion.
void Kobuki_Rotate_Angle(int16_t current_angle_tick)
{
    int16_t speed;
    int16_t error;
    int32_t current_angle_deg;

    // Gyro ticks to degrees.
    current_angle_deg = (int32_t)(
        (double)(current_angle_tick + (int32_t)HALF_TURN_CENTIDEG)
        * DEG_PER_TICK
    );

    // Remaining angle.
    error = (int16_t)(setpoint_angle_deg - current_angle_deg);

    // Take the short way around.
    if (error > DEG_HALF_TURN) {
        error -= DEG_FULL_TURN;
    } else if (error < -DEG_HALF_TURN) {
        error += DEG_FULL_TURN;
    }

    // Done at zero whole-degree error.
    if (error == 0) {
        rotation_complete = 1;
        Kobuki_Rotate(0);
    } else {
        // Faster for a large error, slower near the target.
        speed = (sign(error) * MIN_SPEED)
              + (int16_t)(GAIN_ROTATE * error);

        Kobuki_Rotate(speed);
    }
}

// Provided - project resource (Microcontroller Project Resources, Dr M. Forbes).
void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance)
{
    distance_complete = 0;

    // Requested distance in encoder ticks, added to the current count.
    setpoint_distance_tick =
        current_distance_tick + (int32_t)(distance / MM_PER_TICK);
}

// Adapted from project resource: signed 16-bit wrap instead of setpoint_alias; stops the motor on completion.
void Kobuki_Drive_Distance(uint16_t current_distance_tick)
{
    uint16_t target_tick;
    int32_t error_ticks;
    int16_t error_mm;
    int16_t speed;

    // Target within the encoder's 16-bit range.
    target_tick = (uint16_t)setpoint_distance_tick;

    error_ticks = (int32_t)target_tick
                - (int32_t)current_distance_tick;

    // Encoder wraps between 65535 and 0.
    if (error_ticks > INT16_MAX) {
        error_ticks -= ENCODER_WRAP_TICKS;
    } else if (error_ticks < INT16_MIN) {
        error_ticks += ENCODER_WRAP_TICKS;
    }

    // Remaining ticks to millimetres.
    error_mm = (int16_t)(error_ticks * MM_PER_TICK);

    if (abs(error_mm) <= DISTANCE_TOLERANCE_MM) {
        distance_complete = 1;
        Kobuki_Drive(0);
    } else {
        speed = (sign(error_mm) * MIN_SPEED)
              + (int16_t)(GAIN_DRIVE * error_mm);

        Kobuki_Drive(speed);
    }
}

// Original. R = v/ω; wheel speed scaled by (R ± b/2)/R, b = 230 mm.
void Kobuki_Drive_Arc(int16_t speed, int16_t radius)
{
    int32_t cmd_speed = speed;

    // Arc speed is scaled by (R ± b/2) / R. b is the wheelbase.
    if (radius > ARC_SPIN_RADIUS_MM) {
        cmd_speed = (int32_t)((double)speed * ((double)radius + WHEELBASE_MM / 2.0) / (double)radius);
    } else if (radius < -ARC_SPIN_RADIUS_MM) {
        cmd_speed = (int32_t)((double)speed * ((double)radius - WHEELBASE_MM / 2.0) / (double)radius);
    }

    if (cmd_speed > SPEED_LIMIT) {
        cmd_speed = SPEED_LIMIT;
    } else if (cmd_speed < -SPEED_LIMIT) {
        cmd_speed = -SPEED_LIMIT;
    }

    uint8_t payload[] = {KOBUKI_CMD_BASE_CONTROL, KOBUKI_BASE_CONTROL_SIZE, 0x00, 0x00, 0x00, 0x00};
    payload[2] = (uint8_t)cmd_speed;
    payload[3] = (uint8_t)(cmd_speed >> 8);
    payload[4] = (uint8_t)radius;
    payload[5] = (uint8_t)(radius >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

// Original.
void Kobuki_Heading_Hold_Set(int16_t angle_tick)
{
    hold_angle_tick = angle_tick;
}

// Original. ω = 0.05 × e_θ (deg) rad/s, |ω| ≤ 0.3; R = v/ω.
void Kobuki_Drive_Distance_Hold(uint16_t current_distance_tick, int16_t current_angle_tick)
{
    int32_t error_ticks = (int32_t)(uint16_t)setpoint_distance_tick
                        - (int32_t)current_distance_tick;

    if (error_ticks > INT16_MAX) {
        error_ticks -= ENCODER_WRAP_TICKS;
    } else if (error_ticks < INT16_MIN) {
        error_ticks += ENCODER_WRAP_TICKS;
    }

    int16_t error_mm = (int16_t)(error_ticks * MM_PER_TICK);

    if (abs(error_mm) <= DISTANCE_TOLERANCE_MM) {
        distance_complete = 1;
        Kobuki_Drive(0);
        return;
    }

    int16_t speed = (sign(error_mm) * MIN_SPEED) + (int16_t)(GAIN_DRIVE * error_mm);
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    } else if (speed < -SPEED_LIMIT) {
        speed = -SPEED_LIMIT;
    }

    // Heading error, short way round. +ve means turn left.
    int32_t heading_err = (int32_t)hold_angle_tick - current_angle_tick;
    if (heading_err > HALF_TURN_CENTIDEG) {
        heading_err -= FULL_TURN_CENTIDEG;
    } else if (heading_err < -HALF_TURN_CENTIDEG) {
        heading_err += FULL_TURN_CENTIDEG;
    }

    double turn_rate = HOLD_GAIN_RAD_S_PER_DEG * ((double)heading_err / CENTIDEG_PER_DEG);
    if (turn_rate > HOLD_MAX_RATE_RAD_S) {
        turn_rate = HOLD_MAX_RATE_RAD_S;
    } else if (turn_rate < -HOLD_MAX_RATE_RAD_S) {
        turn_rate = -HOLD_MAX_RATE_RAD_S;
    }

    // radius = speed / turn rate. Sign flips when reversing.
    double radius = (turn_rate != 0.0) ? ((double)speed / turn_rate) : 0.0;

    if ((radius > HOLD_STRAIGHT_RADIUS_MM) || (radius < -HOLD_STRAIGHT_RADIUS_MM) || (turn_rate == 0.0)) {
        Kobuki_Drive_Arc(speed, 0); // error is tiny, drive straight
    } else {
        int16_t radius_mm = (int16_t)radius;
        if ((radius_mm >= -ARC_SPIN_RADIUS_MM) && (radius_mm <= ARC_SPIN_RADIUS_MM)) {
            radius_mm = (radius_mm < 0) ? -ARC_MIN_RADIUS_MM : ARC_MIN_RADIUS_MM; // ±1 means spin on the spot
        }
        Kobuki_Drive_Arc(speed, radius_mm);
    }
}

// Original. d = wrap16(target - current) × MM_PER_TICK.
int16_t Kobuki_Drive_Distance_Remaining(uint16_t current_distance_tick)
{
    int32_t error_ticks = (int32_t)(uint16_t)setpoint_distance_tick
                        - (int32_t)current_distance_tick;

    // Encoder wraps between 65535 and 0.
    if (error_ticks > INT16_MAX) {
        error_ticks -= ENCODER_WRAP_TICKS;
    } else if (error_ticks < INT16_MIN) {
        error_ticks += ENCODER_WRAP_TICKS;
    }

    return (int16_t)(error_ticks * MM_PER_TICK);
}

// Provided - project resource (Microcontroller Project Resources, Dr M. Forbes).
#define EXTERN_PWR 0x30 // GPO bits that keep the external 5 V and 3.3 V outputs on

// Provided - project resource (Microcontroller Project Resources, Dr M. Forbes).
void Kobuki_Update_LEDs(Kobuki_LED_Enum led_1_colour,
                        Kobuki_LED_Enum led_2_colour)
{
    // Keep the external 5 V and 3.3 V outputs on.
    uint8_t GPO_lsb = EXTERN_PWR;

    // Both LED colours in the upper byte.
    uint8_t GPO_msb = (uint8_t)(
        led_1_colour | (led_2_colour << 2)
    );

    uint8_t payload[] = {
        KOBUKI_CMD_GPO,
        KOBUKI_GPO_SIZE,
        GPO_lsb,
        GPO_msb
    };
    Kobuki_Tx(payload, sizeof(payload));
}

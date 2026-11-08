#include <stdint.h>
#include <stdlib.h> // abs()
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"

#define SIZE_FEEDBACK 26

// Feedback packet field offsets.
#define BUMPER 4
#define WHEEL_DROP 5
#define CLIFF 6
#define DIST_LSB 7
#define DIST_MSB 8
#define BUTTON 13
#define ANGLE_LSB 24
#define ANGLE_MSB 25

#define DEG_PER_TICK (1.0 / 100.0)
#define GAIN_ROTATE (1.0 / 0.90)
#define MIN_SPEED ((int16_t)50)

#define sign(x) ((int8_t)((x) >= 0 ? 1 : -1))

#define PI (3.14159)
#define WHEEL_RADIUS (35.0) // mm
#define REVOLUTIONS_PER_TICK ((132.0 / 6545.0) / 52.0)

#define MM_PER_TICK (2.0 * PI * WHEEL_RADIUS * REVOLUTIONS_PER_TICK)

#define GAIN_DRIVE (100.0 / 100.0)

static volatile int32_t setpoint_distance_tick;
static volatile int16_t setpoint_angle_deg;
static volatile int16_t hold_angle_tick;

// Turn rate (rad/s) per degree of heading error. 0 disables hold.
#define HOLD_GAIN_RAD_S_PER_DEG (0.05)
#define HOLD_MAX_RATE_RAD_S (0.30)
#define WHEELBASE_MM (230.0)

uint8_t rotation_complete = 0;
uint8_t distance_complete = 0;

void Kobuki_Tx(uint8_t *payload, uint8_t size_payload)
{
    uint8_t checksum = 0, i;

    UART_Tx(0xAA); // header
    UART_Tx(0x55);
    UART_Tx(size_payload);
    checksum ^= size_payload;

    for (i = 0; i < size_payload; ++i) {
        UART_Tx(payload[i]);
        checksum ^= payload[i];
    }

    UART_Tx(checksum);
}

uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback)
{
    uint8_t feedback_header = 0, size_payload = 0, checksum = 0, rx_checksum = 0;
    uint32_t i;

    // Find the 0xAA 0x55 header.
    while (1) {
        feedback_header = UART_Rx();
        if (feedback_header == 0xAA) {
            feedback_header = UART_Rx();
            if (feedback_header == 0x55) {
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

void Kobuki_Drive(int16_t speed)
{
    uint8_t payload[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00};

    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Rotate(int16_t speed)
{
    uint8_t payload[] = {0x01, 0x04, 0x00, 0x00, 0x01, 0x00};

    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg)
{
    volatile int32_t init_angle;

    rotation_complete = 0;

    // Gyro ticks to a heading in the 0–360 degree range.
    init_angle = (int32_t)(
        (double)(current_angle_tick + (int32_t)18000)
        * DEG_PER_TICK
    );

    // Target heading after the requested rotation.
    setpoint_angle_deg =
        (init_angle + rot_angle_deg + 360) % 360;
}

void Kobuki_Rotate_Angle(int16_t current_angle_tick)
{
    int16_t speed;
    int16_t error;
    int32_t current_angle_deg;

    // Gyro ticks to degrees.
    current_angle_deg = (int32_t)(
        (double)(current_angle_tick + (int32_t)18000)
        * DEG_PER_TICK
    );

    // Remaining angle.
    error = (int16_t)(setpoint_angle_deg - current_angle_deg);

    // Take the short way around.
    if (error > 180) {
        error -= 360;
    }
    else if (error < -180) {
        error += 360;
    }

    // Done at zero whole-degree error.
    if (error == 0) {
        rotation_complete = 1;
        Kobuki_Rotate(0);
    }
    else {
        // Faster for a large error, slower near the target.
        speed = (sign(error) * MIN_SPEED)
              + (int16_t)(GAIN_ROTATE * error);

        Kobuki_Rotate(speed);
    }
}

void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance)
{
    distance_complete = 0;

    // Requested distance in encoder ticks, added to the current count.
    setpoint_distance_tick =
        current_distance_tick + (int32_t)(distance / MM_PER_TICK);
}

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
    if (error_ticks > 32767) {
        error_ticks -= 65536;
    }
    else if (error_ticks < -32768) {
        error_ticks += 65536;
    }

    // Remaining ticks to millimetres.
    error_mm = (int16_t)(error_ticks * MM_PER_TICK);

    if (abs(error_mm) <= 2) {
        distance_complete = 1;
        Kobuki_Drive(0);
    }
    else {
        speed = (sign(error_mm) * MIN_SPEED)
              + (int16_t)(GAIN_DRIVE * error_mm);

        Kobuki_Drive(speed);
    }
}

void Kobuki_Drive_Arc(int16_t speed, int16_t radius)
{
    int32_t cmd_speed = speed;

    // Arc speed is scaled by (R ± b/2) / R. b is the wheelbase.
    if (radius > 1) {
        cmd_speed = (int32_t)((double)speed * ((double)radius + WHEELBASE_MM / 2.0) / (double)radius);
    }
    else if (radius < -1) {
        cmd_speed = (int32_t)((double)speed * ((double)radius - WHEELBASE_MM / 2.0) / (double)radius);
    }

    if (cmd_speed > SPEED_LIMIT) {
        cmd_speed = SPEED_LIMIT;
    }
    else if (cmd_speed < -SPEED_LIMIT) {
        cmd_speed = -SPEED_LIMIT;
    }

    uint8_t payload[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00};
    payload[2] = (uint8_t)cmd_speed;
    payload[3] = (uint8_t)(cmd_speed >> 8);
    payload[4] = (uint8_t)radius;
    payload[5] = (uint8_t)(radius >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Heading_Hold_Set(int16_t angle_tick)
{
    hold_angle_tick = angle_tick;
}

void Kobuki_Drive_Distance_Hold(uint16_t current_distance_tick, int16_t current_angle_tick)
{
    int32_t error_ticks = (int32_t)(uint16_t)setpoint_distance_tick
                        - (int32_t)current_distance_tick;

    if (error_ticks > 32767) {
        error_ticks -= 65536;
    }
    else if (error_ticks < -32768) {
        error_ticks += 65536;
    }

    int16_t error_mm = (int16_t)(error_ticks * MM_PER_TICK);

    if (abs(error_mm) <= 2) {
        distance_complete = 1;
        Kobuki_Drive(0);
        return;
    }

    int16_t speed = (sign(error_mm) * MIN_SPEED) + (int16_t)(GAIN_DRIVE * error_mm);
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT) {
        speed = -SPEED_LIMIT;
    }

    // Heading error, short way round. +ve means turn left.
    int32_t heading_err = (int32_t)hold_angle_tick - current_angle_tick;
    if (heading_err > 18000) {
        heading_err -= 36000;
    }
    else if (heading_err < -18000) {
        heading_err += 36000;
    }

    double turn_rate = HOLD_GAIN_RAD_S_PER_DEG * ((double)heading_err / 100.0);
    if (turn_rate > HOLD_MAX_RATE_RAD_S) {
        turn_rate = HOLD_MAX_RATE_RAD_S;
    }
    else if (turn_rate < -HOLD_MAX_RATE_RAD_S) {
        turn_rate = -HOLD_MAX_RATE_RAD_S;
    }

    // radius = speed / turn rate. Sign flips when reversing.
    double radius = (turn_rate != 0.0) ? ((double)speed / turn_rate) : 0.0;

    if ((radius > 30000.0) || (radius < -30000.0) || (turn_rate == 0.0)) {
        Kobuki_Drive_Arc(speed, 0); // error is tiny, drive straight
    }
    else {
        int16_t r = (int16_t)radius;
        if ((r >= -1) && (r <= 1)) {
            r = (r < 0) ? -2 : 2; // ±1 means spin on the spot
        }
        Kobuki_Drive_Arc(speed, r);
    }
}

int16_t Kobuki_Drive_Distance_Remaining(uint16_t current_distance_tick)
{
    int32_t error_ticks = (int32_t)(uint16_t)setpoint_distance_tick
                        - (int32_t)current_distance_tick;

    // Encoder wraps between 65535 and 0.
    if (error_ticks > 32767) {
        error_ticks -= 65536;
    }
    else if (error_ticks < -32768) {
        error_ticks += 65536;
    }

    return (int16_t)(error_ticks * MM_PER_TICK);
}

#define EXTERN_PWR 0x30

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
        0x0C, // general-purpose output
        0x02, // two data bytes
        GPO_lsb,
        GPO_msb
    };
    Kobuki_Tx(payload, sizeof(payload));
}

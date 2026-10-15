#include <stdint.h>
#include <stdlib.h>  // abs()
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"

#define SIZE_FEEDBACK 26
// Feedback field definitions
#define BUMPER       4
#define WHEEL_DROP   5
#define CLIFF        6
#define DIST_LSB     7
#define DIST_MSB     8
#define BUTTON       13
#define ANGLE_LSB    24
#define ANGLE_MSB    25

#define DEG_PER_TICK (1.0 / 100.0)
#define GAIN_ROTATE  (1.0 / 0.90)
#define MIN_SPEED    ((int16_t)50)

#define sign(x) ((int8_t)((x) >= 0 ? 1 : -1))

#define PI                   (3.14159)
#define WHEEL_RADIUS         (35.0)  // mm
#define REVOLUTIONS_PER_TICK ((132.0 / 6545.0) / 52.0)

#define MM_PER_TICK (2.0 * PI * WHEEL_RADIUS * REVOLUTIONS_PER_TICK)

#define GAIN_DRIVE (100.0 / 100.0)

static volatile int32_t setpoint_distance_tick;
static volatile int16_t setpoint_angle_deg;

/* Updated by the movement controllers we will add next. */
uint8_t rotation_complete = 0;
uint8_t distance_complete = 0;

void Kobuki_Tx(uint8_t *payload, uint8_t size_payload)
{
    uint8_t checksum = 0, i;

    UART_Tx(0xAA); // Send header 0
    UART_Tx(0x55); // Send header 1
    UART_Tx(size_payload); // Send size of payload in bytes
    checksum ^= size_payload; // Calculate running checksum

    // Send each byte of payload one-by-one
    for (i = 0; i < size_payload; ++i) {
        UART_Tx(payload[i]);
        checksum ^= payload[i];
    }

    UART_Tx(checksum); // Send checksum
}

uint8_t Kobuki_Rx(uint8_t *feedback, uint32_t size_feedback)
{
    uint8_t feedback_header = 0, size_payload = 0, checksum = 0, rx_checksum = 0;
    uint32_t i;

    // 1. Find the valid 0xAA 0x55 sequence
    while (1) {
        feedback_header = UART_Rx();
        if (feedback_header == 0xAA) {
            feedback_header = UART_Rx();
            if (feedback_header == 0x55) {
                break; // Valid start found, exit finding loop
            }
        }
    }

    // 2. Read Payload Length, first byte is size of payload inbytes
    size_payload = UART_Rx();
    checksum ^= size_payload; // checksum is XOR ed value ofentire bytestream

    // 3. Determine safe boundary to prevent array overflow
    uint32_t limit = (size_payload < size_feedback) ? size_payload : size_feedback;

    // 4. Read into array of bytestream, this is the basic sensor data
    for (i = 0; i < limit; i++) {
        feedback[i] = UART_Rx();
        checksum ^= feedback[i];
    }

    // 5. Read and discard remaining payload (just for checksum), this is the size of rest of byestream
    for (i = limit; i < size_payload; i++) {
        checksum ^= UART_Rx();
    }

    rx_checksum = UART_Rx(); // checksum from kobuki-final byte
    /* Reject a bad checksum or a payload shorter than required. */
    return ((checksum != rx_checksum) || (size_payload < size_feedback));
}

void Kobuki_Read(Kobuki_Typedef *kobuki)
{
    uint8_t feedback[SIZE_FEEDBACK];
    uint8_t checksum_result;

    checksum_result = Kobuki_Rx(feedback, SIZE_FEEDBACK);

    if (checksum_result == 0U) {
        kobuki->bumper    = feedback[BUMPER];
        kobuki->wheeldrop = feedback[WHEEL_DROP];
        kobuki->cliff     = feedback[CLIFF];
        kobuki->button    = feedback[BUTTON];

        /* Combine the two bytes into a 16-bit encoder count. */
        kobuki->distance =
            (uint16_t)feedback[DIST_LSB] |
            ((uint16_t)feedback[DIST_MSB] << 8);

        /* Combine the two bytes into a signed gyro angle. */
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
    // uint8_t speed_lsb = 0;
    // uint8_t speed_msb = 0;
    // Construct payload
    uint8_t payload[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x00};

    // saturation to avoid going over speed limit
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    // Split LSB & MSB for payload
    // There is no truncation warning due to stdint
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Rotate(int16_t speed)
{
    // uint8_t speed_lsb = 0;
    // uint8_t speed_msb = 0;
    // Construct payload
    uint8_t payload[] = {0x01, 0x04, 0x00, 0x00, 0x01, 0x00};

    // saturation to avoid going over speed limit
    if (speed > SPEED_LIMIT) {
        speed = SPEED_LIMIT;
    }
    else if (speed < -SPEED_LIMIT) {
        speed = (~SPEED_LIMIT) + 1;
    }

    //split lsb & msb for payload
    payload[2] = (uint8_t)speed;
    payload[3] = (uint8_t)(speed >> 8);
    Kobuki_Tx(payload, sizeof(payload));
}

void Kobuki_Rotate_Angle_Setpoint(int16_t current_angle_tick, int16_t rot_angle_deg)
{
    volatile int32_t init_angle;

    rotation_complete = 0;

    /* Convert gyro reading from hundredths of a degree
       into a heading in the 0–360 degree range. */
    init_angle = (int32_t)(
        (double)(current_angle_tick + (int32_t)18000)
        * DEG_PER_TICK
    );

    /* Store the target heading after the requested rotation. */
    setpoint_angle_deg =
        (init_angle + rot_angle_deg + 360) % 360;
}

void Kobuki_Rotate_Angle(int16_t current_angle_tick)
{
    int16_t speed;
    int16_t error;
    int32_t current_angle_deg;

    /* Convert the gyro reading into degrees. */
    current_angle_deg = (int32_t)(
        (double)(current_angle_tick + (int32_t)18000)
        * DEG_PER_TICK
    );

    /* Calculate the remaining angle. */
    error = (int16_t)(setpoint_angle_deg - current_angle_deg);

    /* Adaptation: choose the shortest direction to the target. */
    if (error > 180) {
        error -= 360;
    }
    else if (error < -180) {
        error += 360;
    }

    /* Resource's completion condition: zero whole-degree error. */
    if (error == 0) {
        rotation_complete = 1;

        /* Addition: explicitly stop when the turn finishes. */
        Kobuki_Rotate(0);
    }
    else {
        /* Turn faster for larger errors, slower near the target. */
        speed = (sign(error) * MIN_SPEED)
              + (int16_t)(GAIN_ROTATE * error);

        Kobuki_Rotate(speed);
    }
}

void Kobuki_Drive_Distance_Setpoint(uint16_t current_distance_tick, int16_t distance)
{
    distance_complete = 0;

    /* Convert the requested distance into encoder ticks,
       then add it to the current encoder count. */
    setpoint_distance_tick =
        current_distance_tick + (int32_t)(distance / MM_PER_TICK);
}

void Kobuki_Drive_Distance(uint16_t current_distance_tick)
{
    uint16_t target_tick;
    int32_t error_ticks;
    int16_t error_mm;
    int16_t speed;

    /* Express the target within the encoder's 16-bit range. */
    target_tick = (uint16_t)setpoint_distance_tick;

    error_ticks = (int32_t)target_tick
                - (int32_t)current_distance_tick;

    /* Account for the encoder wrapping between 65535 and 0. */
    if (error_ticks > 32767) {
        error_ticks -= 65536;
    }
    else if (error_ticks < -32768) {
        error_ticks += 65536;
    }

    /* Convert remaining encoder ticks into millimetres. */
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

#define EXTERN_PWR 0x30

void Kobuki_Update_LEDs(Kobuki_LED_Enum led_1_colour,
                        Kobuki_LED_Enum led_2_colour)
{
    /* Keep the external 5 V and 3.3 V power outputs enabled. */
    uint8_t GPO_lsb = EXTERN_PWR;

    /* Pack both LED colours into the upper byte. */
    uint8_t GPO_msb = (uint8_t)(
        led_1_colour | (led_2_colour << 2)
    );

    uint8_t payload[] = {
        0x0C,       // General-purpose output command
        0x02,       // Two data bytes
        GPO_lsb,
        GPO_msb
    };
    Kobuki_Tx(payload, sizeof(payload));
}

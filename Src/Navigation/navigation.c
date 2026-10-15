#include "navigation.h"
#include "kobuki.h"
#define SCAN_SPEED_MM_S ((int16_t)75)

int16_t Navigation_Midpoint(int16_t first_angle, int16_t second_angle)
{
    int32_t difference = (int32_t)second_angle - first_angle;
    // Find the angular difference across the shorter arc.
    if (difference > 18000) {
        difference -= 36000; // If the angle is greater than 180 degrees we minus 360 degrees to get an shorter arc distance e.g. from 190 to -170
    }
    else if (difference < -18000) {
        difference += 36000; // Similar to above, but for the negative case.
    }

    int32_t midpoint = first_angle + difference / 2;

    // Keep the result between -18000 and +17999. Units are measured in 1/100th of a degree.
    if (midpoint >= 18000) {
        midpoint -= 36000;
    }
    else if (midpoint < -18000) {
        midpoint += 36000;
    }

    return (int16_t)midpoint;
}

// Call once when beginning a new scan
void Navigation_Scan_Start(NavigationScan *scan, int16_t current_angle, uint8_t detected)
{
    scan->angle1 = 0;
    scan->angle2 = 0;
    scan->midpoint = 0;
    scan->previous_angle = current_angle;
    scan->swept_angle = 0;

    // If already seeing an object, first find a clear path/direction
    scan->state = (detected != 0U) ? CLEAR_OBJECT : FIND_ANGLE_1;
}


// Call once each main-loop cycle with the latest readings
void Navigation_Scan_Update(NavigationScan *scan, int16_t current_angle, uint8_t detected)
{
    if ((scan->state == SCAN_COMPLETE) || (scan->state == SCAN_FAILED)) {
        Kobuki_Rotate(0);
        return; // No further updates needed
    }

    // Calculate movement since the previous update
    int32_t change = (int32_t)current_angle - scan->previous_angle;

    // Correct heading wraparound
    if (change > 18000) {
        change -= 36000;
    }
    else if (change < -18000) {
        change += 36000;
    }

    // Update the previous angle
    scan->previous_angle = current_angle;
    scan->swept_angle += (change >= 0) ? change : -change;

    // Stop if this step exceeds its permitted scan angle
    int32_t limit = (scan->state == FIND_ANGLE_2) ? 18000 : 36000;

    if (scan->swept_angle >= limit) {
        scan->state = SCAN_FAILED;
        Kobuki_Rotate(0);
        return;
    }

    switch (scan->state) {
        case CLEAR_OBJECT:
        // waits till the object has left then changes the state to finding the angle number 1
            if (detected == 0U) {
                scan->state = FIND_ANGLE_1;
                // Reset the rotation counter so the new scan step gets its own angle limit.
                scan->swept_angle = 0;
            }
            break;

       case FIND_ANGLE_1:
       // Starts recording the angle once it has found the object
            if (detected != 0U) {
                // Save the heading where detection begins
                scan->angle1 = current_angle;
                scan->state = FIND_ANGLE_2;
                scan->swept_angle = 0;
            }
            break;
        case FIND_ANGLE_2:
        // finishes recording once the object leaves the detection zone
            if (detected == 0U) {
                scan->angle2 = current_angle;
                scan->midpoint = Navigation_Midpoint(scan->angle1, scan->angle2);
                scan->state = SCAN_COMPLETE;
                Kobuki_Rotate(0);
                return;
            }
            break;

       case SCAN_COMPLETE:
           break;

       case SCAN_FAILED:
           break;   
   }
   // Continue scanning
   Kobuki_Rotate(SCAN_SPEED_MM_S);
}

// Call once after a successful scan to select the centre heading
void Navigation_Orient_Start(int16_t target_angle) // e.g target angle will be our midpt so pass scan.midpoint
{
    Kobuki_Rotate_Angle_Setpoint(target_angle, 0); // Stores the rotation target as target_angle + 0°
}

// Call each main-loop cycle; return 1 when orientation is complete
uint8_t Navigation_Orient_Update(const Kobuki_Typedef *kobuki)
{
    if (kobuki->rotation_complete != 0U) {
        Kobuki_Rotate(0);
        return 1U;
    }

    Kobuki_Rotate_Angle(kobuki->angle);
    return 0U;
}

///// THIS CODE WILL GO IN THE MAIN FSM FOR SEARCHING BIT when 
// case SEARCH:
//     Navigation_Scan_Update(&scan, my_kobuki.angle, ultrasonic_object.detected);

//     if (scan.state == SCAN_COMPLETE) {
//         /* Use the midpoint heading as the reference with zero offset. */
//         Kobuki_Rotate_Angle_Setpoint(scan.midpoint, 0);
//         state = ORIENT;
//     }
//     else if (scan.state == SCAN_FAILED) {
//         Kobuki_Drive(0);
//         state = IDLE;
//     }
//     break;

// case ORIENT:
//     if (my_kobuki.rotation_complete != 0U) {
//         Kobuki_Rotate(0);

//         /* Next: stationary distance measurement and route selection. */
//     }
//     else {
//         Kobuki_Rotate_Angle(my_kobuki.angle);
//     }
//     break;
// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        reflex.c
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Bump and cliff recovery. On a hit during a forward drive leg:
//              stop, back up, turn away from the hit side, then hand the leg
//              back with the reversed distance added on.
// FSM:         FSM 5 - bump and cliff recovery (report 4.5). Runs before FSM 1 each loop.
// Provenance:  Original.
// ============================================================================

#include "reflex.h"

// Tuning values, chosen on the course (report 3.6, 5.3).
#define BUMP_BACKUP_MM ((int16_t)80) // reverse this far after a bump (mm)
#define BUMP_NUDGE_DEG ((int16_t)7) // then turn this far away from the hit (degrees); raised from 5 to clear the obstacle
#define CLIFF_BACKUP_MM ((int16_t)150) // drop-off: back away further (mm)
#define CLIFF_NUDGE_DEG ((int16_t)5) // degrees; cut from 10, which threw the robot off course
#define REFLEX_MAX_HITS ((uint8_t)6) // per run; more than this means stuck

// FSM 5: Bump and cliff recovery (report 4.5).
typedef enum {
    REFLEX_IDLE,
    REFLEX_BACKUP,
    REFLEX_TURN
} ReflexStep;

static ReflexStep step = REFLEX_IDLE;
static int16_t resume_mm = 0; // forward distance to restore afterwards
static int16_t nudge_deg = 0; // +ve is CCW, -ve is CW
static uint8_t hits = 0U;
static uint8_t centre_turn_right = 1U; // alternates for head-on hits

// Turn away from whichever side was hit: returns -deg (turn right) or +deg (turn left), degrees.
static int16_t nudge_away(uint8_t flags, int16_t deg)
{
    uint8_t left = (flags & SIDE_LEFT) != 0U;
    uint8_t right = (flags & SIDE_RIGHT) != 0U;

    if (left && !right) {
        return (int16_t)-deg; // hit on the left, turn right
    }
    if (right && !left) {
        return deg; // hit on the right, turn left
    }

    // Centre or both sides: no side information, so alternate.
    centre_turn_right = (uint8_t)!centre_turn_right;
    return centre_turn_right ? (int16_t)-deg : deg;
}

// Original.
void Reflex_Reset(void)
{
    step = REFLEX_IDLE;
    hits = 0U;
    resume_mm = 0;
    nudge_deg = 0;
}

// Original.
uint8_t Reflex_Busy(void)
{
    return (step != REFLEX_IDLE) ? 1U : 0U;
}

// Original.
// FSM 5: runs every loop before FSM 1, but only acts on PATHFIND drive legs.
uint8_t Reflex_Update(const Kobuki_Typedef *kobuki, uint8_t driving_forward)
{
    // FSM 5: bump and cliff recovery.
    //   REFLEX_IDLE   - check bumper and cliff on forward drive legs. No hit:
    //                   REFLEX_CLEAR. Hit: stop; REFLEX_GAVE_UP on the seventh
    //                   hit, else set the backup. Next: REFLEX_BACKUP.
    //   REFLEX_BACKUP - reverse 80 mm (bump) or 150 mm (cliff). Next:
    //                   REFLEX_TURN when distance_complete.
    //   REFLEX_TURN   - turn 7 deg (bump) or 5 deg (cliff) away from the hit.
    //                   Next: REFLEX_IDLE when rotation_complete, with the leg
    //                   restarted for d_resume.
    // Every state except a clear REFLEX_IDLE returns REFLEX_ACTIVE.
    switch (step) {
        case REFLEX_IDLE: { // check the bumper and cliff flags
            uint8_t cliff = kobuki->cliff & (SIDE_LEFT | SIDE_CENTRE | SIDE_RIGHT);
            uint8_t bumper = kobuki->bumper & (SIDE_LEFT | SIDE_CENTRE | SIDE_RIGHT);
            int16_t backup_mm;

            if ((driving_forward == 0U) || ((cliff == 0U) && (bumper == 0U))) {
                return REFLEX_CLEAR;
            }

            Kobuki_Drive(0);

            if (++hits > REFLEX_MAX_HITS) {
                return REFLEX_GAVE_UP;
            }

            // Distance left on the interrupted leg.
            int16_t remaining = Kobuki_Drive_Distance_Remaining(kobuki->distance);
            if (remaining < 0) {
                remaining = 0;
            }

            // A cliff takes priority over a bump.
            if (cliff != 0U) {
                backup_mm = CLIFF_BACKUP_MM;
                nudge_deg = nudge_away(cliff, CLIFF_NUDGE_DEG);
            } else {
                backup_mm = BUMP_BACKUP_MM;
                nudge_deg = nudge_away(bumper, BUMP_NUDGE_DEG);
            }

            // Drive the reversed distance again, on top of what was left.
            // d_resume = d_remaining + d_reversed
            resume_mm = (int16_t)(remaining + backup_mm);

            Kobuki_Drive_Distance_Setpoint(kobuki->distance, (int16_t)-backup_mm);
            step = REFLEX_BACKUP;
            return REFLEX_ACTIVE;
        }

        case REFLEX_BACKUP: // reverse 80 mm (bump) or 150 mm (cliff)
            if (!distance_complete) {
                Kobuki_Drive_Distance(kobuki->distance);
            } else {
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, nudge_deg);
                step = REFLEX_TURN;
            }
            return REFLEX_ACTIVE;

        case REFLEX_TURN: // turn 7 deg (bump) or 5 deg (cliff) away from the hit
            if (!rotation_complete) {
                Kobuki_Rotate_Angle(kobuki->angle);
            } else {
                // Hand the leg back, holding the nudged heading until the next turn.
                Kobuki_Heading_Hold_Set(kobuki->angle);
                Kobuki_Drive_Distance_Setpoint(kobuki->distance, resume_mm);
                step = REFLEX_IDLE;
            }
            return REFLEX_ACTIVE;

        default:
            Kobuki_Drive(0);
            step = REFLEX_IDLE;
            return REFLEX_GAVE_UP;
    }
}

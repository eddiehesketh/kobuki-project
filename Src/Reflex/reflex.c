#include "reflex.h"

/* ---- Tuning ---------------------------------------------------------- */
#define BUMP_BACKUP_MM   ((int16_t)80)   /* reverse this far after a bump        */
#define BUMP_NUDGE_DEG   ((int16_t)5)    /* then turn this far away from the hit */
#define CLIFF_BACKUP_MM  ((int16_t)150)  /* drop-off: back away further          */
#define CLIFF_NUDGE_DEG  ((int16_t)10)
#define REFLEX_MAX_HITS  ((uint8_t)6)    /* per run; more than this = stuck      */
/* ---------------------------------------------------------------------- */

typedef enum {
    REFLEX_IDLE,
    REFLEX_BACKUP,
    REFLEX_TURN
} ReflexStep;

static ReflexStep step = REFLEX_IDLE;
static int16_t resume_mm = 0;     /* forward distance to restore afterwards */
static int16_t nudge_deg = 0;     /* +ve = CCW (left), -ve = CW (right)     */
static uint8_t hits = 0U;
static uint8_t centre_turn_right = 1U;  /* alternates for head-on hits      */

/* Turn away from whichever side was hit. */
static int16_t nudge_away(uint8_t flags, int16_t deg)
{
    uint8_t left  = (flags & SIDE_LEFT)  != 0U;
    uint8_t right = (flags & SIDE_RIGHT) != 0U;

    if (left && !right) {
        return (int16_t)-deg;           /* hit on the left  -> turn right */
    }
    if (right && !left) {
        return deg;                     /* hit on the right -> turn left  */
    }

    /* Centre only, or both sides: no side information, so alternate. */
    centre_turn_right = (uint8_t)!centre_turn_right;
    return centre_turn_right ? (int16_t)-deg : deg;
}

void Reflex_Reset(void)
{
    step = REFLEX_IDLE;
    hits = 0U;
    resume_mm = 0;
    nudge_deg = 0;
}

uint8_t Reflex_Busy(void)
{
    return (step != REFLEX_IDLE) ? 1U : 0U;
}

uint8_t Reflex_Update(const Kobuki_Typedef *kobuki, uint8_t driving_forward)
{
    switch (step) {
        case REFLEX_IDLE: {
            uint8_t cliff  = kobuki->cliff  & (SIDE_LEFT | SIDE_CENTRE | SIDE_RIGHT);
            uint8_t bumper = kobuki->bumper & (SIDE_LEFT | SIDE_CENTRE | SIDE_RIGHT);
            int16_t backup_mm;

            if ((driving_forward == 0U) || ((cliff == 0U) && (bumper == 0U))) {
                return REFLEX_CLEAR;
            }

            Kobuki_Drive(0);

            if (++hits > REFLEX_MAX_HITS) {
                return REFLEX_GAVE_UP;
            }

            /* Remember how much of the interrupted leg was left. */
            int16_t remaining = Kobuki_Drive_Distance_Remaining(kobuki->distance);
            if (remaining < 0) {
                remaining = 0;
            }

            /* A drop-off takes priority over a bump. */
            if (cliff != 0U) {
                backup_mm = CLIFF_BACKUP_MM;
                nudge_deg = nudge_away(cliff, CLIFF_NUDGE_DEG);
            }
            else {
                backup_mm = BUMP_BACKUP_MM;
                nudge_deg = nudge_away(bumper, BUMP_NUDGE_DEG);
            }

            /* Drive the reversed distance again on top of what was left. */
            resume_mm = (int16_t)(remaining + backup_mm);

            Kobuki_Drive_Distance_Setpoint(kobuki->distance, (int16_t)-backup_mm);
            step = REFLEX_BACKUP;
            return REFLEX_ACTIVE;
        }

        case REFLEX_BACKUP:
            if (!distance_complete) {
                Kobuki_Drive_Distance(kobuki->distance);
            }
            else {
                Kobuki_Rotate_Angle_Setpoint(kobuki->angle, nudge_deg);
                step = REFLEX_TURN;
            }
            return REFLEX_ACTIVE;

        case REFLEX_TURN:
            if (!rotation_complete) {
                Kobuki_Rotate_Angle(kobuki->angle);
            }
            else {
                /* Hand the drive leg back to the main FSM, holding the nudged
                   heading for the rest of this leg (next path turn snaps back). */
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

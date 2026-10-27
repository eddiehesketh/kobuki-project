#pragma once

#include <stdint.h>
#include "kobuki.h"

/* Kobuki bumper / cliff flag bits (same layout for both, see protocol spec) */
#define SIDE_RIGHT  0x01U
#define SIDE_CENTRE 0x02U
#define SIDE_LEFT   0x04U

/* Return values of Reflex_Update */
#define REFLEX_CLEAR    0U  /* nothing to do, run the main FSM as normal      */
#define REFLEX_ACTIVE   1U  /* recovering, skip the main FSM this loop         */
#define REFLEX_GAVE_UP  2U  /* hit something too many times, go to IDLE        */

/* Clear the recovery state and the hit counter. Call when the run (re)starts. */
void Reflex_Reset(void);

/* 1 while a back-up / nudge manoeuvre is in progress. */
uint8_t Reflex_Busy(void);

/* Call once per main loop, before the main FSM.
   driving_forward: 1 only while a forward drive leg is running.
   On a bumper or cliff hit: stop, reverse, nudge the heading away from the
   side that was hit, then restore the interrupted drive leg (remaining
   distance + the distance reversed) so the main FSM carries on unchanged. */
uint8_t Reflex_Update(const Kobuki_Typedef *kobuki, uint8_t driving_forward);

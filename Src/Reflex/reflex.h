#pragma once

#include <stdint.h>
#include "kobuki.h"

// Bumper and cliff bits. Same layout for both.
#define SIDE_RIGHT 0x01U
#define SIDE_CENTRE 0x02U
#define SIDE_LEFT 0x04U

// Reflex_Update return values.
#define REFLEX_CLEAR 0U // nothing to do
#define REFLEX_ACTIVE 1U // recovering, skip the main FSM
#define REFLEX_GAVE_UP 2U // stuck, go to IDLE

// Clear recovery state and the hit counter. Call when a run starts.
void Reflex_Reset(void);

// 1 while a backup or nudge is in progress.
uint8_t Reflex_Busy(void);

// Call once per loop, before the main FSM.
// driving_forward is 1 only on a forward drive leg.
// On a hit: stop, reverse, nudge away from that side, then resume the leg.
uint8_t Reflex_Update(const Kobuki_Typedef *kobuki, uint8_t driving_forward);

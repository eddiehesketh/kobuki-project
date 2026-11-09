// Student numbers: a1925175 (Charlie Arundell), a1927512 (Angus Heinjus), a1942028 (Thomas Hesketh)
// ============================================================================
// File:        reflex.h
// Authors:     Thomas Hesketh, Charlie Arundell, Angus Heinjus
// Date:        November 2026
// Description: Bump and cliff recovery. On a hit during a forward drive leg:
//              stop, back up, turn away from the hit side, then hand the leg
//              back with the reversed distance added on.
// FSM:         FSM 5 – bump and cliff recovery (report 4.5). Runs before FSM 1 each loop.
// Provenance:  Original.
// ============================================================================

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

// Reflex_Reset
// Provenance: Original.
// Purpose:    Clear recovery state and the hit counter. Call when a run starts.
// Params:     none.
// Returns:    nothing.
// Effects:    no motor command.
void Reflex_Reset(void);

// Reflex_Busy
// Provenance: Original.
// Purpose:    Tell whether a recovery is in progress.
// Params:     none.
// Returns:    1 while a backup or nudge is in progress, else 0.
// Effects:    none.
uint8_t Reflex_Busy(void);

// Reflex_Update
// Provenance: Original.
// Purpose:    One step of FSM 5. Call once per loop, before the main FSM.
//             On a hit: stop, reverse, nudge away from that side, then resume the leg.
// Params:     kobuki          - current robot state.
//             driving_forward - 1 only on a forward drive leg; hits are ignored otherwise.
// Returns:    REFLEX_CLEAR (nothing to do), REFLEX_ACTIVE (recovering, skip
//             the main FSM) or REFLEX_GAVE_UP (seventh hit this run).
// Effects:    sends drive and rotate commands; sets the distance, rotation and
//             heading-hold setpoints.
// Maths:      d_resume = d_remaining + d_reversed (mm).
uint8_t Reflex_Update(const Kobuki_Typedef *kobuki, uint8_t driving_forward);

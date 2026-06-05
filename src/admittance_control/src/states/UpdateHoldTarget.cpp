// =============================================================================
//  states/UpdateHoldTarget.cpp
//
//  Registers UpdateHoldTarget with mc_rtc's FSM state factory so it can be
//  referenced by name in YAML without any additional loader configuration.
// =============================================================================

#include "UpdateHoldTarget.h"
#include <mc_control/fsm/State.h>

// Register the state with the FSM factory under the string "UpdateHoldTarget".
// This makes `base: UpdateHoldTarget` work in YAML states.
EXPORT_SINGLE_STATE("UpdateHoldTarget", mc_control::fsm::UpdateHoldTarget)

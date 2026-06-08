// =============================================================================
//  UpdateHoldTarget.cpp
//  Registers the UpdateHoldTarget state with mc_rtc's FSM factory so it
//  can be referenced by name in YAML without any loader configuration.
// =============================================================================

#include "UpdateHoldTarget.h"
#include <mc_control/fsm/State.h>

EXPORT_SINGLE_STATE("UpdateHoldTarget", UpdateHoldTarget)

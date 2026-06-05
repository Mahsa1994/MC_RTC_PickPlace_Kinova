// =============================================================================
//  states/UpdateHoldTarget.h
//
//  Optional lightweight C++ FSM state.
//
//  PURPOSE
//  -------
//  When the operator releases the arm and presses "Stop Hand-Guiding", the
//  FSM transitions from HandGuide → HoldPosition.  mc_rtc's PostureTask by
//  default re-targets the *reference* configuration (the one set at
//  construction time), NOT the current encoder configuration.  Without this
//  state the arm would drift back to where it was when the controller started.
//
//  This state runs for exactly one control cycle before emitting "OK".  It
//  reads the current encoder positions and writes them as the new PostureTask
//  target, so the arm freezes exactly where the operator left it.
//
//  USAGE IN FSM YAML
//  -----------------
//  Replace the plain MetaTasks-based HoldPosition with:
//
//    HoldPosition:
//      base: UpdateHoldTarget   # ← this state runs first, then MetaTasks
//      tasks:
//        HoldPosture:
//          type: posture
//          stiffness: 10.0
//          weight: 100.0
//
//  Or use it as an interstitial state in the transition map:
//    - [HandGuide,      stopGuiding, SeedPosture,  Auto]
//    - [SeedPosture,    OK,          HoldPosition, Auto]
//
// =============================================================================
#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>

namespace mc_control::fsm
{

/**
 * UpdateHoldTarget
 *
 * One-shot state: reads current joint encoder positions and updates the
 * PostureTask target so the arm holds exactly where it was released.
 */
struct UpdateHoldTarget : State
{
  // -------------------------------------------------------------------------
  void start(Controller & ctl) override
  {
    auto & robot    = ctl.robot();           // command-side robot
    auto & realRobot = ctl.realRobot();      // encoder / observer-side robot

    // Find the PostureTask that is already in the solver.
    // mc_rtc's FSM MetaTasks base state registers tasks under their YAML key.
    // We look for any PostureTask; adjust the name if yours differs.
    mc_tasks::PostureTask * postureTask = nullptr;
    for(auto & t : ctl.solver().tasks())
    {
      postureTask = dynamic_cast<mc_tasks::PostureTask *>(t.get());
      if(postureTask) break;
    }

    if(!postureTask)
    {
      mc_rtc::log::warning(
          "[UpdateHoldTarget] No PostureTask found in solver — "
          "arm may drift back to reference pose after release.");
      output("OK");
      return;
    }

    // Seed the posture target from the REAL robot's current joint positions
    // (as estimated by the encoder observer, gravity-corrected by Kortex).
    const auto & q = realRobot.mbc().q;
    postureTask->posture(q);   // sets ALL joint targets to current encoder values

    mc_rtc::log::success(
        "[UpdateHoldTarget] PostureTask re-seeded from encoders — "
        "arm will hold at released configuration.");

    output("OK");
  }

  // -------------------------------------------------------------------------
  bool run(Controller &) override
  {
    // Completes immediately; the one-shot work was done in start().
    return true;
  }

  // -------------------------------------------------------------------------
  void teardown(Controller &) override {}
};

} // namespace mc_control::fsm

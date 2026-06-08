#pragma once
// =============================================================================
//  UpdateHoldTarget.h
//
//  One-shot FSM state that fires when the operator presses "Stop Hand-Guiding".
//  It runs for exactly one control cycle before emitting "OK".
//
//  PURPOSE
//  -------
//  Without this state, returning to HoldPosition re-targets the PostureTask
//  to whatever joint configuration was active when the controller first
//  started — not where the operator released the arm.
//
//  This state reads ctl.realRobot().mbc().q (encoder-side joint positions,
//  updated each cycle by the Encoder observer) and writes them as the new
//  PostureTask target. The arm then holds exactly at the released pose.
//
//  INTEGRATION
//  -----------
//  The state is registered as "UpdateHoldTarget" by UpdateHoldTarget.cpp.
//  To use it, insert it as an interstitial in the transition map:
//
//    transitions:
//      - [HandGuide,        stopGuiding,  SeedPosture,  Auto]
//      - [SeedPosture,      OK,           HoldPosition, Auto]
//
//  The current KinovaHandGuiding.yaml transitions directly
//  HandGuide → HoldPosition because PostureTask re-reads encoder values
//  automatically at construction when no explicit target is set.
//  Add this state only if you observe the arm snapping back to the
//  startup pose on return to HoldPosition.
// =============================================================================

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>

namespace mc_control::fsm
{

struct UpdateHoldTarget : State
{
  void start(Controller & ctl) override
  {
    // realRobot() is the observer-side robot — joint positions come from
    // the Encoder observer (encoderValues), not from the QP command output.
    const auto & q = ctl.realRobot().mbc().q;

    // Find the PostureTask registered by the HoldPosition state.
    // MetaTasks registers tasks under their YAML key; we search by type.
    mc_tasks::PostureTask * postureTask = nullptr;
    for(auto & t : ctl.solver().tasks())
    {
      postureTask = dynamic_cast<mc_tasks::PostureTask *>(t);
      if(postureTask) { break; }
    }

    if(!postureTask)
    {
      mc_rtc::log::warning("[UpdateHoldTarget] No PostureTask found in solver. "
                           "Arm will hold at controller startup pose, not release pose.");
      output("OK");
      return;
    }

    // Overwrite the posture target with current encoder values.
    // After this call HoldPosition's PostureTask targets the exact
    // joint angles at the moment the operator released the arm.
    postureTask->posture(q);
    mc_rtc::log::success("[UpdateHoldTarget] PostureTask re-seeded from encoder values.");

    output("OK");
  }

  bool run(Controller &) override
  {
    // One-shot: work is done in start(), run() just signals completion.
    return true;
  }

  void teardown(Controller &) override {}
};

} // namespace mc_control::fsm

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>
#include <cmath>

#include "WristSingularity.h"

namespace
{
// Ramp rate for the wrist park. Deliberately slow: the bridge publishes at
// 100 Hz behind a delta_max of 0.05 rad, and the model-vs-real safety gate
// trips at 0.05 rad, so 0.25 rad/s asks for only 0.0025 rad per published
// point - 20x of margin on both. A 1.4 rad park therefore takes ~6 s, which
// is also slow enough for an operator to see it coming and hit the E-stop.
constexpr double kParkRate = 0.25; // rad/s
} // namespace

struct HoldPositionState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override
  {
    postureTask_ = std::make_shared<mc_tasks::PostureTask>(
        ctl.solver(), ctl.robot().robotIndex(), 10.0, 100.0);
    ctl.solver().addTask(postureTask_);

    j5_ = ctl.robot().jointIndexByName("joint_5");

    ctl.gui()->addElement({"Control"},
        mc_rtc::gui::Button("Start Hand-Guiding", [this]() { startRequested_ = true; }),
        mc_rtc::gui::Button("Park Wrist (exit 4/6 singularity)", [this]() { parkRequested_ = true; }),
        mc_rtc::gui::Button("Abort Park", [this]() { parking_ = false; }));

    khg::warnIfSingular(ctl.realRobot(), "HoldPositionState", "at entry");
    mc_rtc::log::success("[HoldPositionState] Holding position. Click to start hand-guiding.");
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    if(parkRequested_)
    {
      parkRequested_ = false;
      if(!parking_)
      {
        parking_ = true;
        mc_rtc::log::warning(
            "[HoldPositionState] PARKING WRIST - joint_5 {:.3f} -> {:+.2f} rad at {:.2f} rad/s. "
            "THE ARM WILL MOVE. Hands clear; click \"Abort Park\" to stop.",
            khg::jointQ(ctl.realRobot(), "joint_5"), khg::kWristParkTarget, kParkRate);
      }
    }

    if(parking_) { stepPark(ctl); }

    if(startRequested_)
    {
      startRequested_ = false;
      // Refuse rather than hand the operator an arm that physically cannot do
      // what hand-guiding promises. Every previous session entered HandGuide
      // near joint_5 = 0 and the rotation complaint was unfalsifiable because
      // of it; the state now cannot be entered from a pose where it must fail.
      if(khg::warnIfSingular(ctl.realRobot(), "HoldPositionState", "refusing to start guiding"))
      {
        mc_rtc::log::error(
            "[HoldPositionState] Hand-guiding NOT started. Click \"Park Wrist\" first "
            "(or move the arm by hand until |joint_5| > {:.2f} rad).",
            khg::kWristSingularBand);
        return false;
      }
      if(parking_)
      {
        mc_rtc::log::warning("[HoldPositionState] Park still in progress - not starting yet.");
        return false;
      }
      output("startGuiding");
      return true;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    ctl.solver().removeTask(postureTask_);
    ctl.gui()->removeCategory({"Control"});
  }

private:
  // Walks the posture target toward kWristParkTarget one timestep at a time.
  // Drives the POSTURE target, not the measured pose: the control robot has to
  // lead the real arm for the bridge to have anything to publish.
  void stepPark(mc_control::fsm::Controller & ctl)
  {
    auto posture = postureTask_->posture();
    const double cur = posture[j5_][0];
    const double err = khg::kWristParkTarget - cur;
    const double step = kParkRate * ctl.timeStep;

    if(std::abs(err) <= step)
    {
      posture[j5_][0] = khg::kWristParkTarget;
      parking_ = false;
      mc_rtc::log::success(
          "[HoldPositionState] Wrist parked: joint_5 target {:+.3f} rad (real {:+.3f}). "
          "Rotational guiding is now conditioned; safe to start hand-guiding.",
          khg::kWristParkTarget, khg::jointQ(ctl.realRobot(), "joint_5"));
    }
    else
    {
      posture[j5_][0] = cur + std::copysign(step, err);
    }
    postureTask_->posture(posture);
  }

  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  size_t j5_ = 0;
  bool startRequested_ = false;
  bool parkRequested_ = false;
  bool parking_ = false;
};

EXPORT_SINGLE_STATE("KHG::HoldPositionState", HoldPositionState)

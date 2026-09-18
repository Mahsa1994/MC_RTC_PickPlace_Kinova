#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>
#include <mc_rtc/gui.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

// Joint-space admittance, replacing the previous Cartesian AdmittanceTask
// (referenced at tool_frame). The bridge now publishes a per-joint,
// gravity/inertia-compensated external torque estimate `tau_ext` (one entry
// per joint in refJointOrder()) directly to the datastore under
// "KHG::tau_ext" instead of collapsing it into a single 6D wrench - see
// kortex_mc_rtc_bridge_admittance.cpp. That collapse was the source of the
// "off-tool-frame push looks like wrist rotation" problem noted in the prior
// version of this file; per-joint torque has no such lever-arm ambiguity.
struct HandGuideState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override
  {
    postureTask_ = std::make_shared<mc_tasks::PostureTask>(
        ctl.solver(), ctl.robot().robotIndex(), 1.0, 1.0);
    ctl.solver().addTask(postureTask_);

    jointNames_ = ctl.robot().refJointOrder();

    // Enter HandGuide already holding: the operator is not pushing yet, so the
    // correct initial behaviour is to keep the pose we arrived at, not to float.
    holding_ = true;
    hold_q_  = ctl.realRobot().mbc().q;

    ctl.gui()->addElement({"Control"},
        mc_rtc::gui::Button("Stop Hand-Guiding", [this]() {
          stopRequested_ = true;
        }),
        mc_rtc::gui::Label("Guide state", [this]() {
          return holding_ ? std::string("HOLD (latched)") : std::string("GUIDING");
        }),
        mc_rtc::gui::NumberInput("max |tau_ext| (Nm)",
            [this]() { return tauNorm_; }, [](double) {}),
        mc_rtc::gui::NumberInput("hold when max|tau| below",
            [this]() { return hold_torque_; }, [this](double v) { hold_torque_ = v; }),
        mc_rtc::gui::NumberInput("guide when max|tau| above",
            [this]() { return guide_torque_; }, [this](double v) { guide_torque_ = v; }),
        mc_rtc::gui::NumberInput("joint admittance (rad/s per Nm)",
            [this]() { return admittance_; }, [this](double v) { admittance_ = v; }),
        mc_rtc::gui::NumberInput("max joint vel (rad/s)",
            [this]() { return maxJointVel_; }, [this](double v) { maxJointVel_ = v; }));

    mc_rtc::log::success("[HandGuideState] Active — push the arm!");
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    // Per-joint external torque estimate from the bridge (already gravity/
    // inertia-compensated and tared/filtered/deadbanded there). Falls back to
    // zero if the bridge hasn't populated it yet.
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(static_cast<int>(jointNames_.size()));
    const auto & tau_ext = ctl.datastore().get<Eigen::VectorXd>("KHG::tau_ext", zero);

    tauNorm_ = tau_ext.size() > 0 ? tau_ext.cwiseAbs().maxCoeff() : 0.0;

    // Single global hold/guide switch for the whole arm (matches how Kinova's
    // own firmware models joint admittance as one mode, not per-joint), with
    // hysteresis (hold_* below, guide_* above) to avoid chattering at the
    // boundary - same reasoning as the previous Cartesian version.
    if(holding_)
    {
      if(tauNorm_ > guide_torque_)
      {
        holding_ = false;
        mc_rtc::log::info("[HandGuideState] GUIDING (max|tau|={:.2f} Nm)", tauNorm_);
      }
    }
    else if(tauNorm_ < hold_torque_)
    {
      holding_ = true;
      hold_q_  = ctl.realRobot().mbc().q;
      mc_rtc::log::info("[HandGuideState] HOLD latched (max|tau|={:.2f} Nm)", tauNorm_);
    }

    if(holding_)
    {
      postureTask_->posture(hold_q_);
    }
    else
    {
      // Direct admittance law: joint velocity proportional to external
      // torque, same first-order form (v = gain * error) as the Cartesian
      // task it replaces. Integrated one step ahead of the MEASURED pose
      // (not the QP's own model) and given to PostureTask::target(), which
      // sidesteps needing raw nrDof-vector ordering for refVel(). Re-anchoring
      // to the measured pose every tick is the same anti-windup reasoning as
      // before: the position error handed to the PD term is always exactly
      // one timestep of the admittance-computed velocity, so it cannot wind
      // up while the operator is pushing.
      const auto & q_meas = ctl.realRobot().mbc().q;
      const double dt = ctl.solver().dt();
      std::map<std::string, std::vector<double>> targets;
      for(size_t i = 0; i < jointNames_.size(); ++i)
      {
        auto idx = ctl.robot().jointIndexByName(jointNames_[i]);
        double qdot = std::clamp(admittance_ * tau_ext[static_cast<int>(i)], -maxJointVel_, maxJointVel_);
        targets[jointNames_[i]] = {q_meas[idx][0] + qdot * dt};
      }
      postureTask_->target(targets);
    }

    if(stopRequested_)
    {
      output("stopGuiding");
      return true;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    ctl.solver().removeTask(postureTask_);
    ctl.gui()->removeCategory({"Control"});
    mc_rtc::log::info("[HandGuideState] Torn down.");
  }

private:
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  std::vector<std::string> jointNames_;
  bool stopRequested_ = false;

  bool holding_ = true;
  std::vector<std::vector<double>> hold_q_;
  double tauNorm_ = 0.0;

  // First-pass gains, shared across all 6 joints (split per-joint later if a
  // specific joint needs it - these are starting points, not measured
  // values). Tune live via the GUI exactly like the Cartesian gains were
  // tuned before.
  double admittance_   = 0.01;  // rad/s per Nm of external torque
  double maxJointVel_  = 0.3;   // rad/s
  double hold_torque_  = 0.5;   // Nm - enter HOLD below this (max over joints)
  double guide_torque_ = 1.0;   // Nm - leave HOLD above this
};

EXPORT_SINGLE_STATE("KHG::HandGuideState", HandGuideState)

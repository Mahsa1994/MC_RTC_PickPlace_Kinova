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

// Joint-space admittance. The bridge publishes a per-joint, gravity/inertia-
// compensated external torque estimate under "KHG::tau_ext" (refJointOrder()
// order). Each tick we turn it into a desired joint velocity and ask the QP
// to track that velocity: target = q_ctl + qdot*dt AND refVel = qdot. With
// Tasks' PostureTask law  qddot = K*(q_t - q) + D*(refVel - qdot)  this makes
// the control robot's velocity converge to qdot with time constant 1/D.
//
// The target is anchored to the CONTROL robot (the QP's own state), not the
// measured one: the real arm is position-controlled and always lags the
// command by its tracking error, so anchoring to the measured pose meant the
// target sat ~one tracking-error behind the model and the QP never moved.
struct HandGuideState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override
  {
    config("stiffness", stiffness_);
    config("damping", damping_);
    config("weight", weight_);
    config("maxJointVel", maxJointVel_);
    config("holdTorque", hold_torque_);
    config("guideTorque", guide_torque_);
    if(config.has("admittance"))
    {
      std::vector<double> a = config("admittance");
      if(a.size() == 6) { for(int i = 0; i < 6; ++i) { admittance_[i] = a[static_cast<size_t>(i)]; } }
      else { mc_rtc::log::error("[HandGuideState] admittance must have 6 entries, got {}", a.size()); }
    }
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    postureTask_ = std::make_shared<mc_tasks::PostureTask>(
        ctl.solver(), ctl.robot().robotIndex(), stiffness_, weight_);
    postureTask_->damping(damping_);
    ctl.solver().addTask(postureTask_);

    jointNames_ = ctl.robot().refJointOrder();
    const auto & mb = ctl.robot().mb();
    nrDof_ = mb.nrDof();
    dofIdx_.clear();
    for(const auto & jn : jointNames_)
    {
      dofIdx_.push_back(mb.jointPosInDof(static_cast<int>(mb.jointIndexByName(jn))));
    }
    qdot_des_ = Eigen::VectorXd::Zero(nrDof_);
    tau_ = Eigen::VectorXd::Zero(static_cast<int>(jointNames_.size()));

    holding_ = true;
    hold_q_  = ctl.robot().mbc().q;
    postureTask_->posture(hold_q_);

    ctl.gui()->addElement({"Control"},
        mc_rtc::gui::Button("Stop Hand-Guiding", [this]() { stopRequested_ = true; }),
        mc_rtc::gui::Label("Guide state", [this]() {
          return holding_ ? std::string("HOLD (latched)") : std::string("GUIDING");
        }),
        mc_rtc::gui::NumberInput("max |tau_ext| (Nm)",
            [this]() { return tauNorm_; }, [](double) {}),
        mc_rtc::gui::NumberInput("hold when max|tau| below",
            [this]() { return hold_torque_; }, [this](double v) { hold_torque_ = v; }),
        mc_rtc::gui::NumberInput("guide when max|tau| above",
            [this]() { return guide_torque_; }, [this](double v) { guide_torque_ = v; }),
        mc_rtc::gui::ArrayInput("joint admittance (rad/s per Nm)", jointNames_,
            [this]() -> const Eigen::Matrix<double, 6, 1> & { return admittance_; },
            [this](const Eigen::Matrix<double, 6, 1> & v) { admittance_ = v; }),
        mc_rtc::gui::NumberInput("max joint vel (rad/s)",
            [this]() { return maxJointVel_; }, [this](double v) { maxJointVel_ = v; }),
        mc_rtc::gui::NumberInput("posture stiffness",
            [this]() { return stiffness_; },
            [this](double v) { stiffness_ = v; postureTask_->stiffness(v); postureTask_->damping(damping_); }),
        mc_rtc::gui::NumberInput("posture damping",
            [this]() { return damping_; },
            [this](double v) { damping_ = v; postureTask_->damping(v); }));

    ctl.logger().addLogEntry("HandGuide_tau_ext", this, [this]() -> const Eigen::VectorXd & { return tau_; });
    ctl.logger().addLogEntry("HandGuide_qdot_des", this, [this]() -> const Eigen::VectorXd & { return qdot_des_; });
    ctl.logger().addLogEntry("HandGuide_holding", this, [this]() { return holding_; });

    mc_rtc::log::success("[HandGuideState] Active (K={}, D={}, w={}, maxVel={} rad/s) - push the arm!",
                         stiffness_, damping_, weight_, maxJointVel_);
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    const Eigen::VectorXd zero = Eigen::VectorXd::Zero(static_cast<int>(jointNames_.size()));
    tau_ = ctl.datastore().get<Eigen::VectorXd>("KHG::tau_ext", zero);
    tauNorm_ = tau_.size() > 0 ? tau_.cwiseAbs().maxCoeff() : 0.0;

    // Single global hold/guide switch with hysteresis so the boundary doesn't chatter.
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
      // Hold the control robot's pose, not the measured one: the arm is
      // position-controlled and will settle onto the command, whereas
      // re-targeting the (lagging) measured pose closes a loop through the
      // hardware that can ring.
      hold_q_ = ctl.robot().mbc().q;
      mc_rtc::log::info("[HandGuideState] HOLD latched (max|tau|={:.2f} Nm)", tauNorm_);
    }

    if(holding_)
    {
      qdot_des_.setZero();
      postureTask_->refVel(qdot_des_);
      postureTask_->posture(hold_q_);
    }
    else
    {
      const auto & q_ctl = ctl.robot().mbc().q;
      const double dt = ctl.solver().dt();
      std::map<std::string, std::vector<double>> targets;
      for(size_t i = 0; i < jointNames_.size(); ++i)
      {
        const int ii = static_cast<int>(i);
        const double qd = std::clamp(admittance_[ii] * tau_[ii], -maxJointVel_, maxJointVel_);
        qdot_des_[dofIdx_[i]] = qd;
        auto idx = ctl.robot().jointIndexByName(jointNames_[i]);
        targets[jointNames_[i]] = {q_ctl[idx][0] + qd * dt};
      }
      postureTask_->refVel(qdot_des_);
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
    ctl.logger().removeLogEntries(this);
    ctl.solver().removeTask(postureTask_);
    ctl.gui()->removeCategory({"Control"});
    mc_rtc::log::info("[HandGuideState] Torn down.");
  }

private:
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  std::vector<std::string> jointNames_;
  std::vector<int> dofIdx_;
  int nrDof_ = 0;
  bool stopRequested_ = false;

  bool holding_ = true;
  std::vector<std::vector<double>> hold_q_;
  double tauNorm_ = 0.0;
  Eigen::VectorXd tau_;
  Eigen::VectorXd qdot_des_;

  // Defaults; override per-controller in KinovaHandGuiding.yaml (HandGuide state block).
  Eigen::Matrix<double, 6, 1> admittance_ = Eigen::Matrix<double, 6, 1>::Constant(0.05); // rad/s per Nm
  double maxJointVel_  = 0.5;    // rad/s
  double hold_torque_  = 0.5;    // Nm - enter HOLD below this (max over joints)
  double guide_torque_ = 1.0;    // Nm - leave HOLD above this
  double stiffness_    = 100.0;
  double damping_      = 20.0;   // velocity tracks refVel with tau = 1/D = 50 ms
  double weight_       = 100.0;
};

EXPORT_SINGLE_STATE("KHG::HandGuideState", HandGuideState)

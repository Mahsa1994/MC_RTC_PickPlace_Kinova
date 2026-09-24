#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>
#include <mc_rtc/gui.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <Eigen/Core>
#include <algorithm>
#include <memory>
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
//
// Stability: tau_ext contains whatever inertial/tracking torque the bridge's
// slow acceleration filter didn't cancel, delayed ~60 ms. Feeding that
// straight back as velocity is a loop whose gain scales with the joint's
// reflected inertia H_ii(q) - largest with the arm fully extended, where a
// 6-7 Hz bang-bang limit cycle on joint_2 was observed. Two mitigations:
//   * velocityTau: first-order lag on the commanded velocity (a virtual
//     inertia), rolling the loop gain off above ~1 Hz;
//   * inertiaScaling: per-joint gain scaled by H_ii(stance)/H_ii(q) so the
//     loop gain stays roughly constant across the workspace.
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
    config("velocityTau", velocityTau_);
    config("inertiaScaling", inertiaScaling_);
    config("inertiaScaleMin", inertiaScaleMin_);
    config("inertiaScaleMax", inertiaScaleMax_);
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
    qdot_f_.setZero();
    inertiaScale_.setOnes();
    hDiag_.setZero();

    fd_ = std::make_unique<rbd::ForwardDynamics>(mb);
    {
      rbd::MultiBodyConfig mbc = ctl.robot().mbc();
      for(const auto & [jn, qv] : ctl.robot().module().stance())
      {
        if(mb.jointIndexByName().count(jn)) { mbc.q[static_cast<size_t>(mb.jointIndexByName(jn))] = qv; }
      }
      rbd::forwardKinematics(mb, mbc);
      rbd::forwardVelocity(mb, mbc);
      fd_->computeH(mb, mbc);
      for(size_t i = 0; i < jointNames_.size(); ++i) { hRef_[static_cast<int>(i)] = fd_->H()(dofIdx_[i], dofIdx_[i]); }
    }

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
        mc_rtc::gui::NumberInput("velocity lag tau (s)",
            [this]() { return velocityTau_; }, [this](double v) { velocityTau_ = std::max(0.0, v); }),
        mc_rtc::gui::Checkbox("inertia scaling", [this]() { return inertiaScaling_; },
            [this]() { inertiaScaling_ = !inertiaScaling_; }),
        mc_rtc::gui::ArrayInput("inertia scale (applied)", jointNames_,
            [this]() -> const Eigen::Matrix<double, 6, 1> & { return inertiaScale_; },
            [](const Eigen::Matrix<double, 6, 1> &) {}),
        mc_rtc::gui::NumberInput("posture stiffness",
            [this]() { return stiffness_; },
            [this](double v) { stiffness_ = v; postureTask_->stiffness(v); postureTask_->damping(damping_); }),
        mc_rtc::gui::NumberInput("posture damping",
            [this]() { return damping_; },
            [this](double v) { damping_ = v; postureTask_->damping(v); }));

    ctl.logger().addLogEntry("HandGuide_tau_ext", this, [this]() -> const Eigen::VectorXd & { return tau_; });
    ctl.logger().addLogEntry("HandGuide_qdot_des", this, [this]() -> const Eigen::VectorXd & { return qdot_des_; });
    ctl.logger().addLogEntry("HandGuide_holding", this, [this]() { return holding_; });
    ctl.logger().addLogEntry("HandGuide_inertia_scale", this, [this]() -> const Eigen::Matrix<double, 6, 1> & { return inertiaScale_; });
    ctl.logger().addLogEntry("HandGuide_H_diag", this, [this]() -> const Eigen::Matrix<double, 6, 1> & { return hDiag_; });

    mc_rtc::log::success("[HandGuideState] Active (K={}, D={}, w={}, maxVel={} rad/s, velTau={} s, inertiaScaling={}) - push the arm!",
                         stiffness_, damping_, weight_, maxJointVel_, velocityTau_, inertiaScaling_);
    mc_rtc::log::info("[HandGuideState] H_ref diag at stance: [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                      hRef_[0], hRef_[1], hRef_[2], hRef_[3], hRef_[4], hRef_[5]);
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
      mc_rtc::log::info("[HandGuideState] HOLD latched (max|tau|={:.2f} Nm)", tauNorm_);
    }

    const auto & mb = ctl.robot().mb();
    const auto & q_ctl = ctl.robot().mbc().q;
    const double dt = ctl.solver().dt();

    if(inertiaScaling_)
    {
      fd_->computeH(mb, ctl.robot().mbc());
      for(size_t i = 0; i < jointNames_.size(); ++i)
      {
        const int ii = static_cast<int>(i);
        hDiag_[ii] = fd_->H()(dofIdx_[i], dofIdx_[i]);
        inertiaScale_[ii] = std::clamp(hRef_[ii] / std::max(hDiag_[ii], 1e-6), inertiaScaleMin_, inertiaScaleMax_);
      }
    }
    else { inertiaScale_.setOnes(); }

    // Same law in both modes: HOLD just drives the torque input to zero so the
    // commanded velocity decays through the lag instead of stopping dead.
    const double lag = velocityTau_ > 0.0 ? dt / (velocityTau_ + dt) : 1.0;
    for(size_t i = 0; i < jointNames_.size(); ++i)
    {
      const int ii = static_cast<int>(i);
      const double tau_in = holding_ ? 0.0 : tau_[ii];
      const double qd_raw = std::clamp(admittance_[ii] * inertiaScale_[ii] * tau_in, -maxJointVel_, maxJointVel_);
      qdot_f_[ii] += lag * (qd_raw - qdot_f_[ii]);
      qdot_des_[dofIdx_[i]] = qdot_f_[ii];
    }

    const bool settled = holding_ && qdot_f_.cwiseAbs().maxCoeff() < 1e-3;
    if(settled)
    {
      if(!wasSettled_) { hold_q_ = q_ctl; }
      qdot_f_.setZero();
      qdot_des_.setZero();
      postureTask_->refVel(qdot_des_);
      postureTask_->posture(hold_q_);
    }
    else
    {
      std::map<std::string, std::vector<double>> targets;
      for(size_t i = 0; i < jointNames_.size(); ++i)
      {
        auto idx = ctl.robot().jointIndexByName(jointNames_[i]);
        targets[jointNames_[i]] = {q_ctl[idx][0] + qdot_f_[static_cast<int>(i)] * dt};
      }
      postureTask_->refVel(qdot_des_);
      postureTask_->target(targets);
    }
    wasSettled_ = settled;

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
  bool wasSettled_ = false;
  std::vector<std::vector<double>> hold_q_;
  double tauNorm_ = 0.0;
  Eigen::VectorXd tau_;
  Eigen::VectorXd qdot_des_;
  Eigen::Matrix<double, 6, 1> qdot_f_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> inertiaScale_ = Eigen::Matrix<double, 6, 1>::Ones();
  Eigen::Matrix<double, 6, 1> hDiag_ = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> hRef_ = Eigen::Matrix<double, 6, 1>::Ones();
  std::unique_ptr<rbd::ForwardDynamics> fd_;

  double velocityTau_     = 0.15;  // s; virtual-inertia lag on commanded velocity
  bool   inertiaScaling_  = true;
  double inertiaScaleMin_ = 0.15;
  double inertiaScaleMax_ = 1.0;

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

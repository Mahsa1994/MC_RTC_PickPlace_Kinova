// ============================================================
//  Generic FSM states for PickPlaceController
//  ------------------------------------------------------------
//  Instead of one class per pick-and-place step, this file
//  provides three reusable, YAML-configured state classes:
//
//    * CartesianMove  — time-parameterized 6-DoF EE motion
//                       (uses BSplineTrajectoryTask; supports
//                       explicit duration and optional waypoints)
//    * JointMove      — joint-space motion for safe postures
//    * Gripper        — open/close with timeout fallback
//    * Idle           — terminal state
//
//  The FSM is assembled purely from the YAML `states:` section,
//  so you can change speeds, trajectories, proximity waypoints,
//  etc. for HRI experiments without recompiling.
// ============================================================

#include "PickPlaceController.h"

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_rtc/logging.h>

#include <SpaceVecAlg/Conversions.h>
#include <algorithm>

// ── Helpers ──────────────────────────────────────────────────────────────────
static PickPlaceController & ppc(mc_control::fsm::Controller & ctl)
{
  return static_cast<PickPlaceController &>(ctl);
}

static sva::PTransformd poseFromConfig(const mc_rtc::Configuration & cfg)
{
  Eigen::Vector3d t   = cfg("translation");
  Eigen::Vector3d rpy = cfg("rotation");
  // Convention: rpy = [roll, pitch, yaw] applied as Z·Y·X
  Eigen::Matrix3d R =
    (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
     Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
  return sva::PTransformd(R, t);
}

// Resolve a "target" block. Two forms are supported:
//   target:
//     translation: [x, y, z]
//     rotation:    [r, p, y]
// OR (relative to a named reference pose defined at top-level YAML):
//   target:
//     ref: pick            # or "place" or "home"
//     offset: [0, 0, 0.15] # optional, added to translation
//     inherit_orientation: false  # optional; if true, keep current EE rotation
static sva::PTransformd resolveTarget(mc_control::fsm::Controller & ctl,
                                      const mc_rtc::Configuration & cfg,
                                      const std::string & ee_frame)
{
  // Form 1: absolute pose
  if(cfg.has("translation") && cfg.has("rotation"))
  {
    return poseFromConfig(cfg);
  }

  // Form 2: reference-based
  std::string ref = cfg("ref");
  sva::PTransformd base;
  if(ref == "home")       base = ppc(ctl).homePose();
  else if(ref == "pick")  base = ppc(ctl).pickPose();
  else if(ref == "place") base = ppc(ctl).placePose();
  else if(ref == "current") base = ctl.robot().frame(ee_frame).position();
  else throw std::runtime_error("[resolveTarget] Unknown reference: " + ref);

  Eigen::Vector3d t = base.translation();
  Eigen::Matrix3d R = base.rotation();

  if(cfg.has("offset"))
  {
    Eigen::Vector3d offset = cfg("offset");
    t += offset;
  }
  if(cfg.has("inherit_orientation"))
  {
    bool inherit = cfg("inherit_orientation");
    if(inherit) R = ctl.robot().frame(ee_frame).position().rotation();
  }

  // Safety: clamp Z
  if(t.z() < ppc(ctl).zMinLimit()) t.z() = ppc(ctl).zMinLimit();

  return sva::PTransformd(R, t);
}

// ════════════════════════════════════════════════════════════════════════════
//  CartesianMove — time-parameterized 6-DoF end-effector motion
// ════════════════════════════════════════════════════════════════════════════
struct CartesianMove : mc_control::fsm::State
{
  // Config (all overridable from YAML)
  double duration_      = 3.0;
  double stiffness_     = 10.0;
  double weight_        = 1000.0;
  double pos_threshold_ = 0.02;   // 2 cm
  double ori_threshold_ = 0.10;   // ~5.7 deg
  double settle_timeout_ = 2.0;   // extra time after duration to converge
  std::string ee_frame_  = "tool_frame";
  std::string next_state_;

  mc_rtc::Configuration target_cfg_;
  std::vector<Eigen::Vector3d> pos_waypoints_;

  // Runtime
  std::shared_ptr<mc_tasks::BSplineTrajectoryTask> traj_;
  sva::PTransformd target_;
  double t_elapsed_ = 0.0;
  double dt_        = 0.01; //0.005;

  // Posture task backup (we temporarily lower its priority so it
  // doesn't fight the Cartesian trajectory).
  double prev_posture_weight_    = 1.0;
  double prev_posture_stiffness_ = 1.0;

  int tick_ = 0;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("duration"))       duration_       = config("duration");
    if(config.has("stiffness"))      stiffness_      = config("stiffness");
    if(config.has("weight"))         weight_         = config("weight");
    if(config.has("pos_threshold"))  pos_threshold_  = config("pos_threshold");
    if(config.has("ori_threshold"))  ori_threshold_  = config("ori_threshold");
    if(config.has("settle_timeout")) settle_timeout_ = config("settle_timeout");
    if(config.has("ee_frame"))       ee_frame_       = static_cast<std::string>(config("ee_frame"));
    if(config.has("next"))           next_state_     = static_cast<std::string>(config("next"));

    target_cfg_ = config("target");

    if(config.has("waypoints"))
    {
      // Convert to vector-of-3-doubles first, then build Eigen::Vector3d
      // explicitly (avoids the Matrix(const T&) constructor ambiguity).
      std::vector<std::vector<double>> wps = config("waypoints");
      pos_waypoints_.clear();
      for(const auto & p : wps)
      {
        if(p.size() >= 3) pos_waypoints_.emplace_back(p[0], p[1], p[2]);
      }
    }
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    dt_        = ctl.solver().dt();
    t_elapsed_ = 0.0;
    tick_      = 0;
    target_    = resolveTarget(ctl, target_cfg_, ee_frame_);

    // Back off the posture task so the QP respects the Cartesian trajectory.
    if(auto pt = ctl.getPostureTask(ctl.robot().name()))
    {
      prev_posture_weight_    = pt->weight();
      prev_posture_stiffness_ = pt->stiffness();
      pt->weight(1.0);
      pt->stiffness(1.0);
    }

    traj_ = std::make_shared<mc_tasks::BSplineTrajectoryTask>(
        ctl.robot().frame(ee_frame_),
        duration_,
        stiffness_,
        weight_,
        target_,
        pos_waypoints_);

    ctl.solver().addTask(traj_);

    auto cur = ctl.robot().frame(ee_frame_).position();
    mc_rtc::log::info("[{}] Cartesian move started (duration={:.2f}s, stiffness={:.1f})",
                      name(), duration_, stiffness_);
    mc_rtc::log::info("[{}]   from: [{:+.3f}, {:+.3f}, {:+.3f}]",
                      name(), cur.translation().x(), cur.translation().y(), cur.translation().z());
    mc_rtc::log::info("[{}]   to:   [{:+.3f}, {:+.3f}, {:+.3f}]",
                      name(), target_.translation().x(), target_.translation().y(), target_.translation().z());
    if(!pos_waypoints_.empty())
      mc_rtc::log::info("[{}]   via {} waypoint(s)", name(), pos_waypoints_.size());
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    t_elapsed_ += dt_;

    // During the scheduled trajectory time, just let it run.
    if(t_elapsed_ < duration_) return false;

    // After the trajectory has been "played out", check convergence.
    auto cur = ctl.robot().frame(ee_frame_).position();
    double pos_err = (cur.translation() - target_.translation()).norm();
    double ori_err = sva::rotationError(cur.rotation(), target_.rotation()).norm();

    if(pos_err < pos_threshold_ && ori_err < ori_threshold_)
    {
      mc_rtc::log::success("[{}] Reached target (pos_err={:.4f} m, ori_err={:.4f} rad)",
                           name(), pos_err, ori_err);
      output(next_state_);
      return true;
    }

    // Periodic progress log while settling
//    static int tick = 0;
    if((tick_++ % 200) == 0)
    {
      mc_rtc::log::warning("[{}] Settling: pos_err={:.4f} m, ori_err={:.4f} rad",
                           name(), pos_err, ori_err);
    }

    // Bail out gracefully if we've been settling too long
    if(t_elapsed_ > duration_ + settle_timeout_)
    {
      mc_rtc::log::error(
          "[{}] Settle timeout after {:.2f}s extra (pos_err={:.4f}, ori_err={:.4f}). Advancing anyway.",
          name(), settle_timeout_, pos_err, ori_err);
      output(next_state_);
      return true;
    }

    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    if(traj_) ctl.solver().removeTask(traj_);

    if(auto pt = ctl.getPostureTask(ctl.robot().name()))
    {
      pt->weight(prev_posture_weight_);
      pt->stiffness(prev_posture_stiffness_);
    }
  }
};

// ════════════════════════════════════════════════════════════════════════════
//  ComplianceCartesianMove — impedance-compliant, time-parameterized EE motion
//  ------------------------------------------------------------
//  Same config surface as CartesianMove (target/waypoints/duration), but
//  tracks the trajectory with an ImpedanceTask instead of a stiff
//  BSplineTrajectoryTask: if a human pushes the arm off the planned path,
//  the trajectory clock pauses (the moving target stops advancing) until
//  the measured wrench (already deadbanded/clamped upstream by
//  kortex_mc_rtc_bridge_impedance) clears for `clear_hold_time` seconds,
//  then resumes toward the same waypoints/target it was already headed to.
//  This is the state that should carry the object across the workspace
//  while a human is nearby - use plain CartesianMove for legs where no
//  human interaction is expected (e.g. returning home at the very start).
// ════════════════════════════════════════════════════════════════════════════
struct ComplianceCartesianMove : mc_control::fsm::State
{
  // Trajectory config (mirrors CartesianMove)
  double duration_       = 3.0;
  double pos_threshold_  = 0.02;
  double ori_threshold_  = 0.10;
  double settle_timeout_ = 2.0;
  std::string ee_frame_  = "tool_frame";
  std::string next_state_;
  mc_rtc::Configuration target_cfg_;
  std::vector<Eigen::Vector3d> pos_waypoints_;

  // Kinematic tracking gains (how fast the compliant pose chases its target)
  double task_stiffness_ = 20.0;
  double task_damping_   = 10.0;
  double task_weight_    = 100.0;

  // Impedance (virtual mass-spring-damper) gains - same shape as
  // ImpedanceHoldState's `gains:` block in impedance_control.
  mc_rtc::Configuration gains_config_;

  // Contact-pause behavior. contact_force_threshold_ must stay ABOVE
  // whatever `deadband_force` the bridge is launched with, since anything
  // that survives the bridge's deadband is already a real, non-trivial
  // wrench - see kortex_mc_rtc_bridge_impedance.cpp.
  double contact_force_threshold_ = 8.0;  // N
  double clear_hold_time_         = 0.3;  // s of sustained clear force before resuming

  // Runtime
  std::shared_ptr<mc_tasks::force::ImpedanceTask> task_;
  std::vector<sva::PTransformd> waypts_; // full chain: start -> waypoints -> target
  std::vector<double> seg_duration_;     // per-segment duration (sums to duration_)
  double t_elapsed_   = 0.0;   // trajectory clock - only advances while clear of contact
  double clear_timer_ = 0.0;
  bool   paused_      = false;
  double dt_          = 0.01;
  int    tick_        = 0;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("duration"))       duration_       = config("duration");
    if(config.has("pos_threshold"))  pos_threshold_  = config("pos_threshold");
    if(config.has("ori_threshold"))  ori_threshold_  = config("ori_threshold");
    if(config.has("settle_timeout")) settle_timeout_ = config("settle_timeout");
    if(config.has("ee_frame"))       ee_frame_       = static_cast<std::string>(config("ee_frame"));
    if(config.has("next"))           next_state_     = static_cast<std::string>(config("next"));
    if(config.has("stiffness"))      task_stiffness_ = config("stiffness");
    if(config.has("damping"))        task_damping_   = config("damping");
    if(config.has("weight"))         task_weight_    = config("weight");
    if(config.has("gains"))          gains_config_   = config("gains");
    if(config.has("contact_force_threshold")) contact_force_threshold_ = config("contact_force_threshold");
    if(config.has("clear_hold_time"))         clear_hold_time_         = config("clear_hold_time");

    target_cfg_ = config("target");

    if(config.has("waypoints"))
    {
      std::vector<std::vector<double>> wps = config("waypoints");
      pos_waypoints_.clear();
      for(const auto & p : wps)
        if(p.size() >= 3) pos_waypoints_.emplace_back(p[0], p[1], p[2]);
    }
  }

  static double quinticS(double u)
  {
    u = std::clamp(u, 0.0, 1.0);
    return 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    dt_          = ctl.solver().dt();
    t_elapsed_   = 0.0;
    clear_timer_ = 0.0;
    paused_      = false;
    tick_        = 0;

    sva::PTransformd final_target = resolveTarget(ctl, target_cfg_, ee_frame_);
    sva::PTransformd start_pose   = ctl.robot().frame(ee_frame_).position();

    // Build the waypoint chain: start -> intermediate waypoints -> final
    // target. Intermediate waypoints inherit the final target's orientation
    // (same convention as CartesianMove's waypoints).
    waypts_.clear();
    waypts_.push_back(start_pose);
    for(const auto & wp : pos_waypoints_)
      waypts_.push_back(sva::PTransformd(final_target.rotation(), wp));
    waypts_.push_back(final_target);

    // Split duration_ across segments proportionally to chord length, so a
    // longer leg gets a proportionally longer slice of time.
    std::vector<double> chord(waypts_.size() - 1, 0.0);
    double total_chord = 0.0;
    for(size_t i = 0; i + 1 < waypts_.size(); ++i)
    {
      chord[i] = (waypts_[i + 1].translation() - waypts_[i].translation()).norm();
      total_chord += chord[i];
    }
    seg_duration_.assign(chord.size(), duration_ / std::max<size_t>(1, chord.size()));
    if(total_chord > 1e-6)
      for(size_t i = 0; i < chord.size(); ++i)
        seg_duration_[i] = duration_ * (chord[i] / total_chord);

    task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(
        ctl.robot().frame(ee_frame_), task_stiffness_, task_weight_);
    task_->cutoffPeriod(0.05);
    task_->stiffness(task_stiffness_);
    task_->damping(task_damping_);
    task_->targetPose(start_pose);

    auto & gains = task_->gains();
    auto readV3 = [&](const char * grp, const char * key, const Eigen::Vector3d & def) -> Eigen::Vector3d {
      if(gains_config_.has(grp) && gains_config_(grp).has(key))
      {
        Eigen::Vector3d v = gains_config_(grp)(key);
        return v;
      }
      return def;
    };
    gains.mass().linear(   readV3("mass",      "linear",  {3, 3, 3}));
    gains.mass().angular(  readV3("mass",      "angular", {2, 2, 2}));
    gains.spring().linear( readV3("stiffness", "linear",  {800, 800, 800}));
    gains.spring().angular(readV3("stiffness", "angular", {40, 40, 40}));
    gains.damper().linear( readV3("damping",   "linear",  {120, 120, 120}));
    gains.damper().angular(readV3("damping",   "angular", {30, 30, 30}));
    gains.wrench().linear( readV3("wrench",    "linear",  {1, 1, 1}));
    gains.wrench().angular(readV3("wrench",    "angular", {1, 1, 1}));

    ctl.solver().addTask(task_);

    mc_rtc::log::info("[{}] Compliant move started (duration={:.2f}s, {} waypoint(s))",
                      name(), duration_, pos_waypoints_.size());
  }

  // Evaluate the moving target pose at trajectory-clock time `t`.
  sva::PTransformd targetAt(double t) const
  {
    double acc = 0.0;
    for(size_t i = 0; i < seg_duration_.size(); ++i)
    {
      double segT = seg_duration_[i];
      if(t <= acc + segT || i + 1 == seg_duration_.size())
      {
        double u = segT > 1e-6 ? (t - acc) / segT : 1.0;
        double s = quinticS(u);
        const auto & a = waypts_[i];
        const auto & b = waypts_[i + 1];
        Eigen::Vector3d trans = (1.0 - s) * a.translation() + s * b.translation();
        Eigen::Quaterniond qa(a.rotation());
        Eigen::Quaterniond qb(b.rotation());
        Eigen::Quaterniond q = qa.slerp(s, qb);
        return sva::PTransformd(q.toRotationMatrix(), trans);
      }
      acc += segT;
    }
    return waypts_.back();
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    // Contact gate: pause the trajectory clock while the measured wrench
    // says the arm is in contact, resume only after it's been clear for
    // clear_hold_time_ (avoids chattering pause/resume at the threshold).
    double f = task_->measuredWrench().force().norm();
    if(f > contact_force_threshold_)
    {
      paused_      = true;
      clear_timer_ = 0.0;
    }
    else if(paused_)
    {
      clear_timer_ += dt_;
      if(clear_timer_ >= clear_hold_time_) paused_ = false;
    }

    if(!paused_) t_elapsed_ += dt_;

    task_->targetPose(targetAt(std::min(t_elapsed_, duration_)));

    // SIGN CHECK (safe under dry_run - no hardware command involved).
    // task_->compliancePose() is the ImpedanceTask's own internal target:
    // where the impedance model thinks the end-effector should be, given the
    // measured wrench (same signal ImpedanceHoldState used for its
    // "Deflection" log). It updates from the compliance law regardless of
    // dry_run, since the task doesn't know the resulting command is being
    // dropped. Comparing it to the fixed hold anchor tells you the direction
    // the controller is actually trying to move in response to the measured
    // wrench, without ever touching the real arm.
    // Expected: deflection should point the SAME way as measured force while
    // pushing (yielding), then relax back toward zero after release. If it
    // points the OPPOSITE way from the push, that's a sign error - stop and
    // do not proceed to dry_run:false until it's fixed.
    if((tick_ % 20) == 0)
    {
      Eigen::Vector3d deflection = task_->compliancePose().translation() - waypts_.back().translation();
      Eigen::Vector3d meas_f     = task_->measuredWrench().force();
      Eigen::Vector3d meas_m     = task_->measuredWrench().couple();
      mc_rtc::log::info(
        "[{}] SIGN CHECK -- deflection (m): ({:+.4f}, {:+.4f}, {:+.4f}) | measured force (N): ({:+.3f}, {:+.3f}, {:+.3f}) | measured moment (Nm): ({:+.3f}, {:+.3f}, {:+.3f})",
        name(), deflection.x(), deflection.y(), deflection.z(), meas_f.x(), meas_f.y(), meas_f.z(), meas_m.x(), meas_m.y(), meas_m.z());
    }

    if(t_elapsed_ < duration_) { tick_++; return false; }

    auto cur = ctl.robot().frame(ee_frame_).position();
    double pos_err = (cur.translation() - waypts_.back().translation()).norm();
    double ori_err = sva::rotationError(cur.rotation(), waypts_.back().rotation()).norm();

    if(pos_err < pos_threshold_ && ori_err < ori_threshold_)
    {
      mc_rtc::log::success("[{}] Reached target (pos_err={:.4f} m, ori_err={:.4f} rad)",
                           name(), pos_err, ori_err);
      output(next_state_);
      return true;
    }

    if((tick_++ % 200) == 0)
    {
      mc_rtc::log::warning("[{}] Settling: pos_err={:.4f} m, ori_err={:.4f} rad{}",
                           name(), pos_err, ori_err, paused_ ? " (paused - contact)" : "");
    }

    if(t_elapsed_ > duration_ + settle_timeout_)
    {
      mc_rtc::log::error("[{}] Settle timeout after {:.2f}s extra (pos_err={:.4f}, ori_err={:.4f}). Advancing anyway.",
                         name(), settle_timeout_, pos_err, ori_err);
      output(next_state_);
      return true;
    }

    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    if(task_) ctl.solver().removeTask(task_);
  }
};

// ════════════════════════════════════════════════════════════════════════════
//  JointMove — joint-space motion (useful for safe home postures)
// ════════════════════════════════════════════════════════════════════════════
struct JointMove : mc_control::fsm::State
{
  std::map<std::string, std::vector<double>> target_joints_;
  double duration_   = 3.0;
  double stiffness_  = 2.0;
  double weight_     = 100.0;
  double threshold_  = 0.15;
  std::string next_state_;

  double t_elapsed_           = 0.0;
  double dt_                  = 0.01; //0.005;
  double prev_weight_         = 1.0;
  double prev_stiffness_      = 1.0;

  int tick_ = 0;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("duration"))  duration_  = config("duration");
    if(config.has("stiffness")) stiffness_ = config("stiffness");
    if(config.has("weight"))    weight_    = config("weight");
    if(config.has("threshold")) threshold_ = config("threshold");
    if(config.has("next"))      next_state_ = static_cast<std::string>(config("next"));

    // target: [v1, v2, ... v6]  (array form only — simplest and most common)
    if(config.has("target"))
    {
      std::vector<double> vals = config("target");
      for(size_t i = 0; i < vals.size(); ++i)
      {
        target_joints_["joint_" + std::to_string(i + 1)] = {vals[i]};
      }
    }
  }

/*  void start(mc_control::fsm::Controller & ctl) override
  {
    dt_        = ctl.solver().dt();
    t_elapsed_ = 0.0;
    tick_      = 0;

    auto pt = ctl.getPostureTask(ctl.robot().name());
    if(!pt)
    {
      mc_rtc::log::error("[{}] No posture task available!", name());
      output(next_state_);
      return;
    }

// ── DEBUG: print actual joint names the robot knows ──────────────
    mc_rtc::log::info("[{}] Robot joint names:", name());
    for(const auto & j : ctl.robot().mb().joints())
    {
      mc_rtc::log::info("[{}]   '{}' (dof={})", name(), j.name(), j.dof());
    }

// ── DEBUG: print what WE are commanding ──────────────────────────
    mc_rtc::log::info("[{}] Our target_joints_:", name());
    for(const auto & kv : target_joints_)
      mc_rtc::log::info("[{}]   '{}' = {:.4f}", name(), kv.first, kv.second[0]);


    prev_weight_    = pt->weight();
    prev_stiffness_ = pt->stiffness();
    pt->stiffness(stiffness_);
    pt->weight(weight_);
    pt->target(target_joints_);

    mc_rtc::log::info("[{}] Joint-space move started (duration={:.2f}s)", name(), duration_);
  } */

void start(mc_control::fsm::Controller & ctl) override
  {
    dt_        = ctl.solver().dt();
    t_elapsed_ = 0.0;
    tick_      = 0;

    auto pt = ctl.getPostureTask(ctl.robot().name());
    if(!pt)
    {
      mc_rtc::log::error("[{}] No posture task available!", name());
      output(next_state_);
      return;
    }

    // ── DEBUG: print actual joint names the robot knows ──────────────
    mc_rtc::log::info("[{}] Robot joint names:", name());
    for(const auto & j : ctl.robot().mb().joints())
      mc_rtc::log::info("[{}]   '{}' (dof={})", name(), j.name(), j.dof());

    // ── FIX: wrap each target angle to within π of the current q ─────
    // Prevents commanding a 360° detour when the bridge seeds joints in
    // a different wrap than the YAML value (e.g. +263° → −97°).
    const auto & q   = ctl.robot().mbc().q;
    const auto & mbs = ctl.robot().mb().joints();
    for(size_t ji = 0; ji < mbs.size(); ++ji)
    {
      const std::string & jname = mbs[ji].name();
      if(mbs[ji].dof() != 1) continue;
      if(!target_joints_.count(jname)) continue;

      double current = q[ji][0];
      double & target = target_joints_.at(jname)[0];
      // Snap to the nearest equivalent angle
      while(target - current >  M_PI) target -= 2.0 * M_PI;
      while(target - current < -M_PI) target += 2.0 * M_PI;
    }

    // ── DEBUG: print current q vs wrapped target ──────────────────────
    mc_rtc::log::info("[{}] Joint current_q vs target (after wrap):", name());
    for(size_t ji = 0; ji < mbs.size(); ++ji)
    {
      const std::string & jname = mbs[ji].name();
      if(mbs[ji].dof() != 1) continue;
      double current = q[ji][0];
      double target  = target_joints_.count(jname) ? target_joints_.at(jname)[0] : current;
      mc_rtc::log::info("[{}]   '{}' current={:.4f}  target={:.4f}  delta={:.4f}",
                        name(), jname, current, target, target - current);
    }

    prev_weight_    = pt->weight();
    prev_stiffness_ = pt->stiffness();
    pt->stiffness(stiffness_);
    pt->weight(weight_);
    pt->target(target_joints_);

    mc_rtc::log::info("[{}] Joint-space move started (duration={:.2f}s)", name(), duration_);
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    t_elapsed_ += dt_;
    auto pt = ctl.getPostureTask(ctl.robot().name());
    if(!pt) { output(next_state_); return true; }

    double err = pt->eval().norm();

    if(t_elapsed_ >= duration_ && err < threshold_)
    {
      mc_rtc::log::success("[{}] Joint target reached (err={:.4f}).", name(), err);
      output(next_state_);
      return true;
    }

//    static int tick = 0;
    if((tick_++ % 200) == 0)
    {
      mc_rtc::log::info("[{}] err={:.4f}", name(), err);
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    if(auto pt = ctl.getPostureTask(ctl.robot().name()))
    {
      pt->weight(prev_weight_);
      pt->stiffness(prev_stiffness_);
    }
  }
};

// ════════════════════════════════════════════════════════════════════════════
//  Gripper — open/close with timeout fallback
// ════════════════════════════════════════════════════════════════════════════

/*struct Gripper : mc_control::fsm::State
{
  std::string action_    = "close";
  double      timeout_   = 5.0;
  std::string next_state_;

  bool   sent_      = false;
  double t_elapsed_ = 0.0;
  double dt_        = 0.005;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("action"))  action_  = static_cast<std::string>(config("action"));
    if(config.has("timeout")) timeout_ = config("timeout");
    if(config.has("next"))    next_state_ = static_cast<std::string>(config("next"));
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    dt_        = ctl.solver().dt();
    t_elapsed_ = 0.0;
    ppc(ctl).resetGripperDone();
    sent_ = ppc(ctl).sendGripperGoal(action_);
    // sent_ = true;
    mc_rtc::log::info("[{}] Gripper action: {}", name(), action_);
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    t_elapsed_ += dt_;

    if(sent_ && ppc(ctl).isGripperDone())
    {
      mc_rtc::log::success("[{}] Gripper done.", name());
      output(next_state_);
      return true;
    }
    if(t_elapsed_ >= timeout_)
    {
      mc_rtc::log::warning("[{}] Gripper timeout ({:.1f}s), proceeding.", name(), timeout_);
      output(next_state_);
      return true;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller &) override {}
}; */


struct Gripper : mc_control::fsm::State
{
  std::string action_    = "close";
  double      timeout_   = 5.0; // Increased default timeout slightly to allow ROS 2 discovery
  std::string next_state_;

  bool   sent_      = false;
  double t_elapsed_ = 0.0;
  double dt_        = 0.005;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("action"))  action_  = static_cast<std::string>(config("action"));
    if(config.has("timeout")) timeout_ = config("timeout");
    if(config.has("next"))    next_state_ = static_cast<std::string>(config("next"));
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    dt_        = ctl.solver().dt();
    t_elapsed_ = 0.0;
    ppc(ctl).resetGripperDone();
    
    // We do NOT send the goal on start() because the DDS discovery might not be ready.
    sent_ = false; 
    mc_rtc::log::info("[{}] Gripper state initialized. Waiting to establish connection for action: {}", name(), action_);
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    t_elapsed_ += dt_;

    // Attempt to send the goal to ROS 2 until the action server is discovered and ready
    if(!sent_)
    {
      sent_ = ppc(ctl).sendGripperGoal(action_);
      if(sent_)
      {
        mc_rtc::log::info("[{}] Connection established. Gripper action successfully sent: {}", name(), action_);
      }
    }

    // Monitor for the completion callback once sent
    if(sent_ && ppc(ctl).isGripperDone())
    {
      mc_rtc::log::success("[{}] Gripper done.", name());
      output(next_state_);
      return true;
    }

    // Safety timeout in case of physical/network jams
    if(t_elapsed_ >= timeout_)
    {
      mc_rtc::log::warning("[{}] Gripper timeout ({:.1f}s), proceeding.", name(), timeout_);
      output(next_state_);
      return true;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller &) override {}
};






// ════════════════════════════════════════════════════════════════════════════
//  Idle — terminal state
// ════════════════════════════════════════════════════════════════════════════
struct Idle : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller &) override
  {
    mc_rtc::log::success("[Idle] Pick-and-place complete.");
  }
  bool run(mc_control::fsm::Controller &) override { return false; }
  void teardown(mc_control::fsm::Controller &) override {}
};

// ════════════════════════════════════════════════════════════════════════════
//  State factory — ONE extern "C" block for all state classes.
// ════════════════════════════════════════════════════════════════════════════
extern "C"
{
  __attribute__((visibility("default")))
  void MC_RTC_FSM_STATE(std::vector<std::string> & names)
  {
    names = {"CartesianMove", "ComplianceCartesianMove", "JointMove", "Gripper", "Idle"};
  }

  __attribute__((visibility("default")))
  mc_control::fsm::State * create(const std::string & name)
  {
    if(name == "CartesianMove")           return new CartesianMove();
    if(name == "ComplianceCartesianMove") return new ComplianceCartesianMove();
    if(name == "JointMove")               return new JointMove();
    if(name == "Gripper")                 return new Gripper();
    if(name == "Idle")                    return new Idle();
    return nullptr;
  }

  __attribute__((visibility("default")))
  void destroy(mc_control::fsm::State * ptr)
  {
    delete ptr;
  }
}

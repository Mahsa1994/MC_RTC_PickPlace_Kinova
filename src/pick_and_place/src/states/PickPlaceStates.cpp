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
#include <mc_rtc/logging.h>

#include <SpaceVecAlg/Conversions.h>

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
struct Gripper : mc_control::fsm::State
{
  std::string action_    = "close";
  double      timeout_   = 2.0;
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
    ppc(ctl).sendGripperGoal(action_);
    sent_ = true;
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
    names = {"CartesianMove", "JointMove", "Gripper", "Idle"};
  }

  __attribute__((visibility("default")))
  mc_control::fsm::State * create(const std::string & name)
  {
    if(name == "CartesianMove") return new CartesianMove();
    if(name == "JointMove")     return new JointMove();
    if(name == "Gripper")       return new Gripper();
    if(name == "Idle")          return new Idle();
    return nullptr;
  }

  __attribute__((visibility("default")))
  void destroy(mc_control::fsm::State * ptr)
  {
    delete ptr;
  }
}

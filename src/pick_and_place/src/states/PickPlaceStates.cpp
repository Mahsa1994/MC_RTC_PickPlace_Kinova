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
  else if(ref == "current") base = ctl.realRobot().frame(ee_frame).position();
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
    if(inherit) R = ctl.realRobot().frame(ee_frame).position().rotation();
  }

  // Safety: clamp Z
  if(t.z() < ppc(ctl).zMinLimit()) t.z() = ppc(ctl).zMinLimit();

  return sva::PTransformd(R, t);
}

// Resync the QP-controlled robot's configuration to the real,
// encoder-observed one before planning a new trajectory. Without this,
// ctl.robot() can carry over drift accumulated during a previous state
// (dry_run, or the bridge's model-vs-real safety gate blocking publishing
// for a while) - so a freshly-built trajectory, and any `ref: current`
// target, would be planned from wherever the internal model drifted to
// rather than from the arm's actual physical pose. Suspected root cause of
// the 2026-08-19 "unexpected direction" MoveHome incident: the arm was
// physically at Home, but if ctl.robot() had already drifted elsewhere,
// the trajectory built in start() would path from that fictional point.
static void resyncControlToReal(mc_control::fsm::Controller & ctl)
{
  ctl.robot().mbc().q     = ctl.realRobot().mbc().q;
  ctl.robot().mbc().alpha = ctl.realRobot().mbc().alpha;
  ctl.robot().forwardKinematics();
  ctl.robot().forwardVelocity();
}

// ════════════════════════════════════════════════════════════════════════════
//  CartesianMove — time-parameterized 6-DoF end-effector motion
// ════════════════════════════════════════════════════════════════════════════
struct CartesianMove : mc_control::fsm::State
{
  // Config (all overridable from YAML)
  double duration_      = 3.0;   // MINIMUM duration - see effective_duration_/v_max_* below
  double stiffness_     = 10.0;
  double weight_        = 1000.0;
  double pos_threshold_ = 0.02;   // 2 cm
  double ori_threshold_ = 0.10;   // ~5.7 deg
  double settle_timeout_ = 2.0;   // extra time after duration to converge
  std::string ee_frame_  = "tool_frame";
  std::string next_state_;

  // Hard caps on peak Cartesian velocity, enforced by construction via
  // effective_duration_ below - same reasoning/derivation as JointMove's
  // v_max_ (2026-08-26). Defaults chosen with the same ~4x margin under the
  // bridge's delta_max-implied real-tracking ceiling.
  double v_max_lin_ = 0.05;   // m/s
  double v_max_ang_ = 0.05;   // rad/s

  mc_rtc::Configuration target_cfg_;
  std::vector<Eigen::Vector3d> pos_waypoints_;

  // Runtime
  std::shared_ptr<mc_tasks::BSplineTrajectoryTask> traj_;
  sva::PTransformd target_;
  double t_elapsed_          = 0.0;
  double dt_                 = 0.01; //0.005;
  double effective_duration_ = 3.0;  // = max(duration_, time needed so peak vel <= v_max_*)

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
    if(config.has("v_max_lin"))      v_max_lin_      = config("v_max_lin");
    if(config.has("v_max_ang"))      v_max_ang_      = config("v_max_ang");

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

    resyncControlToReal(ctl);
    target_    = resolveTarget(ctl, target_cfg_, ee_frame_);

    auto cur = ctl.robot().frame(ee_frame_).position();

    // BUG FOUND 2026-08-26: CartesianMove had the same missing-speed-bound
    // gap that caused MoveHome's 2026-08-24 stall (see JointMove's
    // v_max_/effective_duration_ note) - MoveHome was fixed by moving to
    // JointMove, but ReturnHome (also CartesianMove, never migrated) hit the
    // identical failure mode live: after MoveToPick's incomplete
    // convergence left the arm farther from home than usual, ReturnHome's
    // fixed-duration BSplineTrajectoryTask requested a peak velocity past
    // what the bridge's delta_max lets the real arm track, tripped
    // model_real_gate almost immediately, and then stalled forever - since
    // nothing resyncs ctl.robot() mid-run, the internal model just finished
    // its pre-planned spline in simulation while the real arm sat frozen.
    // Fixed the same way as JointMove: duration_ is now a MINIMUM, stretched
    // (effective_duration_) so peak velocity stays under v_max_lin_/
    // v_max_ang_ by construction (1.875 = quintic peak/avg factor).
    double pos_delta = (target_.translation() - cur.translation()).norm();
    double ori_delta = sva::rotationError(cur.rotation(), target_.rotation()).norm();
    effective_duration_ = std::max({duration_,
                                     1.875 * pos_delta / std::max(v_max_lin_, 1e-6),
                                     1.875 * ori_delta / std::max(v_max_ang_, 1e-6)});

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
        effective_duration_,
        stiffness_,
        weight_,
        target_,
        pos_waypoints_);

    ctl.solver().addTask(traj_);

    mc_rtc::log::info(
        "[{}] Cartesian move started - effective duration {:.2f}s (configured min {:.2f}s), "
        "v_max_lin={:.3f} m/s, v_max_ang={:.3f} rad/s, pos_delta={:.4f} m, ori_delta={:.4f} rad, "
        "stiffness={:.1f}",
        name(), effective_duration_, duration_, v_max_lin_, v_max_ang_, pos_delta, ori_delta, stiffness_);
    mc_rtc::log::info("[{}]   from: [{:+.3f}, {:+.3f}, {:+.3f}]",
                      name(), cur.translation().x(), cur.translation().y(), cur.translation().z());
    mc_rtc::log::info("[{}]   to:   [{:+.3f}, {:+.3f}, {:+.3f}]",
                      name(), target_.translation().x(), target_.translation().y(), target_.translation().z());
    if(!pos_waypoints_.empty())
      mc_rtc::log::info("[{}]   via {} waypoint(s)", name(), pos_waypoints_.size());

    // ROTATION VERIFICATION (2026-08-19, safe under dry_run - no motion
    // involved, this only reads state). eulerAngles(2,1,0) decomposes as
    // Rz(a)*Ry(b)*Rx(c), the SAME composition poseFromConfig() uses for
    // YAML `rotation: [roll, pitch, yaw]` - so [c,b,a] here is directly
    // comparable to that field, in degrees for comparison against Kinova's
    // web-app thetaX/thetaY/thetaZ. Use this to confirm the config's
    // rotation matches physical reality BEFORE ever trusting a Cartesian
    // target live - see the home_pose incident in the README/YAML.
    {
      const double r2d = 180.0 / M_PI;
      Eigen::Vector3d cur_ea = cur.rotation().eulerAngles(2, 1, 0);
      Eigen::Vector3d tgt_ea = target_.rotation().eulerAngles(2, 1, 0);
      mc_rtc::log::info(
          "[{}]   from rotation [roll,pitch,yaw] (deg): [{:+.2f}, {:+.2f}, {:+.2f}]",
          name(), cur_ea.z() * r2d, cur_ea.y() * r2d, cur_ea.x() * r2d);
      mc_rtc::log::info(
          "[{}]   to   rotation [roll,pitch,yaw] (deg): [{:+.2f}, {:+.2f}, {:+.2f}]",
          name(), tgt_ea.z() * r2d, tgt_ea.y() * r2d, tgt_ea.x() * r2d);
    }

    // JOINT SNAPSHOT (2026-08-24, safe under dry_run - reads realRobot()
    // only, no motion). Gives an unambiguous joint-space ground truth for
    // "where is the arm right now", independent of the Cartesian
    // rotation/IK-feasibility questions above. If this state is started
    // with the arm physically at Kinova's web-app Home, this is exactly
    // Home's true joint configuration - compare against the target this
    // state resolved to (translation/rotation above), and against any
    // joint's URDF limit, without going through IK at all.
    {
      const double r2d = 180.0 / M_PI;
      const auto & q   = ctl.realRobot().mbc().q;
      const auto & mbs = ctl.realRobot().mb().joints();
      mc_rtc::log::info("[{}]   real joints now:", name());
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        mc_rtc::log::info("[{}]     '{}' = {:+.4f} rad ({:+.1f} deg)",
                          name(), mbs[ji].name(), q[ji][0], q[ji][0] * r2d);
      }
    }
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    t_elapsed_ += dt_;

    // During the scheduled trajectory time, just let it run.
    if(t_elapsed_ < effective_duration_) return false;

    // After the trajectory has been "played out", check convergence -
    // against the REAL arm (realRobot()), not the QP-internal model
    // (ctl.robot()), which the task drives toward target_ regardless of
    // whether the real arm actually gets there. See resyncControlToReal()
    // above for why these two can diverge, and the 2026-08-19 incidents
    // this caused.
    auto cur = ctl.realRobot().frame(ee_frame_).position();
    double pos_err = (cur.translation() - target_.translation()).norm();
    double ori_err = sva::rotationError(cur.rotation(), target_.rotation()).norm();

    if(pos_err < pos_threshold_ && ori_err < ori_threshold_)
    {
      mc_rtc::log::success("[{}] Reached target (pos_err={:.4f} m, ori_err={:.4f} rad)",
                           name(), pos_err, ori_err);
      output(next_state_);
      return true;
    }

    // Periodic progress log while settling. Past settle_timeout_ this does
    // NOT force a transition (see note below) - only the message escalates,
    // so a stuck arm is loud, not silent.
    bool past_deadline = t_elapsed_ > effective_duration_ + settle_timeout_;
    if((tick_++ % 200) == 0)
    {
      if(past_deadline)
        mc_rtc::log::error(
            "[{}] NOT converged {:.2f}s past schedule (pos_err={:.4f} m, ori_err={:.4f} rad) - "
            "holding here, will NOT advance until the real arm actually reaches the target.",
            name(), t_elapsed_ - effective_duration_ - settle_timeout_, pos_err, ori_err);
      else
        mc_rtc::log::warning("[{}] Settling: pos_err={:.4f} m, ori_err={:.4f} rad",
                             name(), pos_err, ori_err);
    }

    // NOTE (2026-08-19): this used to force-advance to next_state_ after
    // settle_timeout_ ("advancing anyway"). CartesianMove is what MoveHome
    // uses, and the whole point of MoveHome is guaranteeing the arm is
    // actually at a known pose before the rest of the pipeline runs from it
    // (MoveToPick/etc., and any `ref: current` target, trust that). A
    // silent forced-advance defeats that guarantee exactly when it matters
    // most - unreachable target, real hardware fault. Hold instead; this
    // requires operator attention (loud error above) rather than composing
    // further motion on top of an unverified pose.
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
  double duration_       = 3.0;   // MINIMUM duration - see effective_duration_/v_max_* below
  double pos_threshold_  = 0.02;
  double ori_threshold_  = 0.10;
  double settle_timeout_ = 2.0;
  std::string ee_frame_  = "tool_frame";
  std::string next_state_;
  mc_rtc::Configuration target_cfg_;
  std::vector<Eigen::Vector3d> pos_waypoints_;

  // Hard caps on peak Cartesian velocity - same fix/reasoning applied to
  // CartesianMove above (2026-08-26): the live MoveToPick test showed real
  // joint_5 chronically lagging the model (up to 0.04 rad, never closing)
  // because the moving target here had no speed bound either, so it could
  // ask for more velocity than the bridge's delta_max lets the real arm
  // track. effective_duration_ stretches duration_ (a MINIMUM) so the
  // planned path's peak velocity stays under v_max_lin_/v_max_ang_.
  double v_max_lin_       = 0.05;   // m/s
  double v_max_ang_       = 0.05;   // rad/s
  double effective_duration_ = 3.0;

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
    if(config.has("v_max_lin"))               v_max_lin_               = config("v_max_lin");
    if(config.has("v_max_ang"))               v_max_ang_               = config("v_max_ang");

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

    resyncControlToReal(ctl);
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

    double ori_delta = sva::rotationError(start_pose.rotation(), final_target.rotation()).norm();
    effective_duration_ = std::max({duration_,
                                     1.875 * total_chord / std::max(v_max_lin_, 1e-6),
                                     1.875 * ori_delta / std::max(v_max_ang_, 1e-6)});

    seg_duration_.assign(chord.size(), effective_duration_ / std::max<size_t>(1, chord.size()));
    if(total_chord > 1e-6)
      for(size_t i = 0; i < chord.size(); ++i)
        seg_duration_[i] = effective_duration_ * (chord[i] / total_chord);

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

    mc_rtc::log::info(
        "[{}] Compliant move started - effective duration {:.2f}s (configured min {:.2f}s), "
        "v_max_lin={:.3f} m/s, v_max_ang={:.3f} rad/s, path_len={:.4f} m, ori_delta={:.4f} rad, "
        "{} waypoint(s)",
        name(), effective_duration_, duration_, v_max_lin_, v_max_ang_, total_chord, ori_delta,
        pos_waypoints_.size());
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

    task_->targetPose(targetAt(std::min(t_elapsed_, effective_duration_)));

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
      // WORLD-FRAME sign check. ctl.robot().frame(...).position() is always
      // world-frame by mc_rtc convention (plain forward kinematics) - no
      // ambiguity, unlike measuredWrench()/deltaCompliancePose() whose exact
      // frame conventions we couldn't fully pin down from the header alone,
      // and which produced confusing results across several attempts. Judge
      // this by comparing world_dev's sign against the PHYSICAL direction
      // you push in world terms (e.g. "I pushed straight down toward the
      // floor" -> world_dev.z should go negative) - not against measured
      // force, which is in a different (surface) frame and not directly
      // comparable axis-by-axis.
      Eigen::Vector3d world_dev = ctl.robot().frame(ee_frame_).position().translation()
                                 - waypts_.back().translation();
      mc_rtc::log::info(
        "[{}] SIGN CHECK -- world_dev (m): ({:+.4f}, {:+.4f}, {:+.4f})  [+x=world +X, +y=world +Y, +z=world UP]",
        name(), world_dev.x(), world_dev.y(), world_dev.z());
    }

    if(t_elapsed_ < effective_duration_) { tick_++; return false; }

    // Convergence judged against the REAL arm, not the QP-internal model -
    // see resyncControlToReal() and CartesianMove::run() above. NOTE: the
    // settle-timeout below still force-advances on non-convergence
    // (unchanged from before this review) - MoveHome's equivalent no
    // longer does this, see the note there; flagged for a decision on
    // whether MoveToPick/MoveToPlace should match.
    auto cur = ctl.realRobot().frame(ee_frame_).position();
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

      // DIAGNOSTIC (2026-08-26): a live MoveToPick stall showed pos_err/
      // ori_err and the SIGN CHECK world_dev completely static (not slowly
      // converging) while measured wrench was zero the whole time - i.e.
      // an equilibrium, not a timing problem (the effective_duration_ fix
      // above had no effect on this failure). Real joints snapshot added
      // here to check directly whether a joint is pegged at/near its
      // kinematic limit (a hard QP constraint no amount of extra time can
      // overcome) - e.g. ReturnHome's post-mortem snapshot showed the real
      // arm left at joint_3=-2.4840 rad, only 0.086 rad from its -2.57 rad
      // limit.
      const double r2d = 180.0 / M_PI;
      const auto & rq   = ctl.realRobot().mbc().q;
      const auto & mbs  = ctl.realRobot().mb().joints();
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        mc_rtc::log::info("[{}]   real '{}' = {:+.4f} rad ({:+.1f} deg)",
                          name(), mbs[ji].name(), rq[ji][0], rq[ji][0] * r2d);
      }
    }

    if(t_elapsed_ > effective_duration_ + settle_timeout_)
    {
      // FORCED ADVANCE (2026-08-24): still transitions on non-convergence,
      // unlike CartesianMove/MoveHome (which now holds instead - see the
      // note there), so a stuck compliant move can't hang an experiment
      // session mid-trial. But this means the arm may not actually be at
      // the target (e.g. CloseGripper could close on empty air) - tagged
      // distinctly and consistently ("FORCED ADVANCE") so it's greppable in
      // saved session logs and never reads as an ordinary "Reached target"
      // success.
      mc_rtc::log::error(
          "[{}] FORCED ADVANCE -> {}: NOT converged after {:.2f}s extra "
          "(pos_err={:.4f} m, ori_err={:.4f} rad) - target may not have been "
          "reached; treat this trial/cycle as suspect.",
          name(), next_state_, settle_timeout_, pos_err, ori_err);
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
  double duration_   = 3.0;   // MINIMUM duration - see effective_duration_/v_max_ below
  double stiffness_  = 2.0;   // now a pure TRACKING gain (how tightly the posture task follows
                               // the moving reference below), not a speed control - see note in start()
  double weight_     = 100.0;
  double threshold_  = 0.15;
  // Hard cap (rad/s) on any joint's PEAK commanded velocity, enforced by
  // construction via the quintic time-scaling below - see start(). Default
  // chosen with a 4x margin under the bridge's default delta_max-implied
  // real-tracking ceiling (~0.2 rad/s @ delta_max=0.002, 100Hz publish);
  // override in YAML per-launch-config if delta_max changes.
  double v_max_      = 0.05;
  std::string next_state_;

  double t_elapsed_           = 0.0;
  double dt_                  = 0.01; //0.005;
  double prev_weight_         = 1.0;
  double prev_stiffness_      = 1.0;
  double effective_duration_  = 3.0;  // = max(duration_, time needed so peak velocity <= v_max_)
  std::map<std::string, double> q_start_;

  int tick_ = 0;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("duration"))  duration_  = config("duration");
    if(config.has("stiffness")) stiffness_ = config("stiffness");
    if(config.has("weight"))    weight_    = config("weight");
    if(config.has("threshold")) threshold_ = config("threshold");
    if(config.has("v_max"))     v_max_     = config("v_max");
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

  // Same quintic (minimum-jerk) time-scaling as ComplianceCartesianMove
  // above - 10u^3-15u^4+6u^5, zero velocity/acceleration at both ends.
  static double quinticS(double u)
  {
    u = std::clamp(u, 0.0, 1.0);
    return 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
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

    resyncControlToReal(ctl);

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
    // Reads realRobot() directly (rather than relying on the resync above)
    // so this stays correct even if the resync call is ever reordered.
    const auto & q   = ctl.realRobot().mbc().q;
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

    // ── DEBUG: print current q vs wrapped target, capture q_start_ ────
    // and compute effective_duration_ so peak velocity is bounded by
    // construction (2026-08-26): a live test showed the previous
    // "set target once, let the posture task's spring converge" approach
    // move far faster than intended even after cutting stiffness_ 40x
    // (observed ~0.4-0.9 rad/s vs. an estimated ~0.14 rad/s) - either the
    // stiffness->velocity relationship assumed for that estimate was wrong,
    // or the changed value didn't take effect (same class of stale-build
    // mismatch hit earlier with dry_run; left unresolved since this fix
    // sidesteps the question entirely). Below, q(t) is an explicit quintic
    // interpolation from q_start_ to target_joints_ over effective_duration_
    // - a pure function of time and the captured start/target values, not
    // dependent on any task gain - so its peak velocity
    // (1.875 * max|delta| / effective_duration_, the standard quintic
    // peak-velocity factor) is a guaranteed property, not an estimate, and
    // is directly verifiable in a dry-run log this time (unlike the
    // stiffness-based approach, this profile doesn't depend on real
    // feedback at all past q_start_).
    q_start_.clear();
    double max_abs_delta = 0.0;
    mc_rtc::log::info("[{}] Joint current_q vs target (after wrap):", name());
    for(size_t ji = 0; ji < mbs.size(); ++ji)
    {
      const std::string & jname = mbs[ji].name();
      if(mbs[ji].dof() != 1) continue;
      double current = q[ji][0];
      double target  = target_joints_.count(jname) ? target_joints_.at(jname)[0] : current;
      mc_rtc::log::info("[{}]   '{}' current={:.4f}  target={:.4f}  delta={:.4f}",
                        name(), jname, current, target, target - current);
      if(target_joints_.count(jname))
      {
        q_start_[jname] = current;
        max_abs_delta = std::max(max_abs_delta, std::abs(target - current));
      }
    }
    effective_duration_ = std::max(duration_, 1.875 * max_abs_delta / std::max(v_max_, 1e-6));

    prev_weight_    = pt->weight();
    prev_stiffness_ = pt->stiffness();
    pt->stiffness(stiffness_);
    pt->weight(weight_);
    // BUG FOUND 2026-08-26: this used to be pt->target(target_joints_) (the
    // RAW, un-interpolated final target), on the assumption that run()'s
    // first tick would immediately overwrite it with the s=0 interpolated
    // value, making it "harmless". Two live/dry-run tests with a large
    // single-joint delta (2.38 rad, then 3.08 rad) both hit
    // "[error] QP failed to run()" immediately after this line, before any
    // run()-tick output ever printed - proving the QP's first solve sees
    // THIS raw target, several radians away, not the interpolated one.
    // Smaller deltas (<=0.62 rad, tested earlier) apparently stayed within
    // whatever numerical tolerance the solver has; large ones don't. Fixed
    // by setting the initial target to q_start_ (equivalent to s=0, "stay
    // where you are") - exactly what run()'s first tick computes anyway,
    // removing the momentary large-jump exposure entirely.
    {
      std::map<std::string, std::vector<double>> initial;
      for(const auto & kv : q_start_) initial[kv.first] = {kv.second};
      pt->target(initial);
    }

    mc_rtc::log::info(
        "[{}] Joint-space move started - effective duration {:.2f}s (configured min {:.2f}s), "
        "v_max={:.4f} rad/s, max |delta|={:.4f} rad -> peak velocity {:.4f} rad/s",
        name(), effective_duration_, duration_, v_max_, max_abs_delta,
        1.875 * max_abs_delta / effective_duration_);

    // CARTESIAN POSE SNAPSHOT (2026-08-26, safe under dry_run - reads
    // realRobot() only, no motion). JointMove has no Cartesian awareness of
    // its own (pure joint-space), unlike CartesianMove's "from:"/rotation
    // log - added here specifically to capture pick_pose/place_pose the
    // same proven way home_pose was fixed: physically position the arm,
    // read THIS log, not the Kinova web app's reported pose (confirmed
    // twice now to not correspond to mc_rtc's tool_frame directly). Same
    // eulerAngles(2,1,0) decomposition poseFromConfig() uses, so these
    // numbers drop straight into a YAML `translation:`/`rotation:` block
    // with no conversion needed.
    {
      const double r2d = 180.0 / M_PI;
      auto cur = ctl.realRobot().frame("tool_frame").position();
      Eigen::Vector3d cur_ea = cur.rotation().eulerAngles(2, 1, 0);
      mc_rtc::log::info("[{}]   real tool_frame translation: [{:+.4f}, {:+.4f}, {:+.4f}]",
                        name(), cur.translation().x(), cur.translation().y(), cur.translation().z());
      mc_rtc::log::info(
          "[{}]   real tool_frame rotation [roll,pitch,yaw] (deg): [{:+.2f}, {:+.2f}, {:+.2f}] "
          "-> YAML rotation: [{:.4f}, {:.4f}, {:.4f}]",
          name(), cur_ea.z() * r2d, cur_ea.y() * r2d, cur_ea.x() * r2d,
          cur_ea.z(), cur_ea.y(), cur_ea.x());
    }
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    t_elapsed_ += dt_;
    auto pt = ctl.getPostureTask(ctl.robot().name());
    if(!pt) { output(next_state_); return true; }

    // Feed the interpolated (slow, bounded) reference every tick - the
    // posture task's stiffness_/weight_ now only controls how tightly it
    // tracks THIS moving reference, not how fast the motion itself is.
    double u = effective_duration_ > 1e-6 ? t_elapsed_ / effective_duration_ : 1.0;
    double s = quinticS(u);
    std::map<std::string, std::vector<double>> interp;
    for(const auto & kv : target_joints_)
    {
      double start = q_start_.count(kv.first) ? q_start_.at(kv.first) : kv.second[0];
      interp[kv.first] = {(1.0 - s) * start + s * kv.second[0]};
    }
    pt->target(interp);

    // Convergence judged against the REAL arm (realRobot()), not
    // pt->eval() (the posture task's own ctl.robot()-internal error) -
    // same reasoning as CartesianMove/ComplianceCartesianMove above.
    double err = 0.0;
    {
      const auto & q   = ctl.realRobot().mbc().q;
      const auto & mbs = ctl.robot().mb().joints();
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        const std::string & jname = mbs[ji].name();
        if(!target_joints_.count(jname)) continue;
        double d = q[ji][0] - target_joints_.at(jname)[0];
        err += d * d;
      }
      err = std::sqrt(err);
    }

    if(t_elapsed_ >= effective_duration_ && err < threshold_)
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

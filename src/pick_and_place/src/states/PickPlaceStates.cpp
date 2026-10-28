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
#include <limits>
#include <cmath>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_rtc/logging.h>

#include <SpaceVecAlg/Conversions.h>
#include <algorithm>

// ── Wrap-aware joint difference (2026-10-12) ──────────────────────────────
// joint_1, joint_4 and joint_6 are CONTINUOUS. The driver reports them
// wrapped into [-pi, pi], while the model interpolates a target that
// start() has already unwrapped to be within pi of the start angle (see
// the wrap block in JointMove::start()). The moment the real joint crosses
// the branch cut, a raw `model - real` therefore reads ~2*pi even though
// the two are physically at the SAME place.
//
// That is not hypothetical. Live run 2026-10-12, start_1, joint_1 going
// +1.6002 -> -2.9995 the short way (i.e. increasing through +pi):
//     [start_1] err=0.2459                             <- one tick before
//     [start_1] STALLED - model-vs-real 6.2848 rad on 'joint_1'
//     [start_1] err=6.4266
//     model=-3.1387 rad, real=-3.1406 rad              <- 0.002 rad apart!
// 6.2848 is 2*pi. The stall guard then chattered in and out every tick,
// the convergence error sat at 6.43 against a 0.05 threshold so the state
// could never finish, and the posture task was driven toward a point a
// full turn away - commanded velocity spiked to 0.2678 rad/s against a
// v_max of 0.05 and the arm started travelling the long way round.
//
// Every runtime angle comparison must therefore go through this. Wrapping
// cannot mask a real fault: the stall guard trips at 0.03 rad and the
// bridge's publish gate at 0.05, both orders of magnitude below the pi
// where wrapping starts to have any effect at all.
static inline double angDiff(double a, double b)
{
  double d = a - b;
  while(d >  M_PI) d -= 2.0 * M_PI;
  while(d < -M_PI) d += 2.0 * M_PI;
  return d;
}

// Only these joints spin freely (URDF type `continuous`), so only these may
// have a TARGET wrapped by 2*pi. Wrapping a limited joint sends it the wrong
// way, past its limit. Live 2026-10-28, after the arm was left with the elbow
// folded the other way (joint_3 -2.18) for a hardware change: start_2's
// joint_3 target +1.8801 was "wrapped" to 1.8801 - 2*pi = -4.4031, beyond the
// -2.57 limit, and the move started folding the arm into its own body.
// angDiff() for COMPARING two nearby angles stays fine for every joint.
static inline bool isContinuous(const std::string & joint)
{
  return joint == "joint_1" || joint == "joint_4" || joint == "joint_6";
}

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
    // GLOBAL SPEED KNOB (2026-09-23): `speed_scale` in the YAML, one number
    // per trial condition instead of editing v_max on every state. Note it
    // divides `duration` as well as multiplying v_max - `duration` is a FLOOR,
    // so a leg pinned by it (MoveUpFromPick is, at 15s vs its 14s v_max term)
    // would otherwise ignore the scale entirely and the cycle would not
    // actually speed up. Clamped to [0.1, 3.0] by the controller.
    const double sc_ = ppc(ctl).speedScale();
    effective_duration_ = std::max({duration_ / sc_,
                                     1.875 * pos_delta / std::max(v_max_lin_ * sc_, 1e-6),
                                     1.875 * ori_delta / std::max(v_max_ang_ * sc_, 1e-6)});

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

  // ADDED 2026-09-18: contact detection on the MOMENT channel.
  // A live HoverToPlace test (hand placed in the arm's path) showed the
  // contact registering ONLY here - the estimator reported
  //   RAW  force (0.069, 0.773, 0.367) N   moment (-0.879, 0.959, -1.065) Nm
  //   Est. force (0.00, 0.00, 0.00) N      moment (-0.83, 0.80, -0.97) Nm
  // i.e. raw force norm ~0.86 N, which is BELOW the bridge's 1.0 N
  // deadband_force and is therefore zeroed before the task ever sees it.
  // Force-based detection was structurally dead: no value of
  // contact_force_threshold_ could ever have fired, and the arm ran on
  // through the obstruction until model-vs-real tripped the safety gate.
  // The moment channel by contrast is close to binary here - ~1.5 Nm norm
  // while blocked, exactly 0.00 otherwise - so 0.5 Nm sits with wide margin
  // on both sides. Keep this ABOVE the bridge's deadband_moment only in the
  // sense that anything under the deadband reads as exactly zero anyway.
  double contact_moment_threshold_ = 0.5;  // Nm

  // SPEED-TRACKING THRESHOLD (2026-09-23). The wrench estimate's no-contact
  // baseline grows LINEARLY with joint speed because the bridge's inverse
  // dynamics models gravity, Coriolis and inertia but NOT joint friction.
  // Measured on this arm over speed_scale 1.0/1.5/2.0:
  //     baseline_moment ~= 0.41 + 11.5 * max|qd|   (Nm, R^2 = 0.976)
  // With a fixed 0.5 Nm threshold that baseline false-triggers from about
  // speed_scale 2.0, and by 2.2 it exceeds a real human hand contact
  // (measured 1.50 Nm) - i.e. detection stops being possible at all.
  //
  // Raising the constant instead would have to clear 2.02 Nm to survive
  // speed_scale 3.0, which is ABOVE a real contact: that disables detection
  // rather than reducing it. Scaling the threshold with speed keeps the
  // margin to a real contact CONSTANT across the whole speed range, so
  // speed can be varied as an experimental factor without the compliance
  // behaviour varying with it.
  //
  // Set this to the measured friction slope (Nm per rad/s). 0 = fixed
  // threshold, the pre-2026-09-23 behaviour.
  // RE-MEASURE IT WHENEVER THE PAYLOAD CHANGES - a container on the gripper
  // shifts the INTERCEPT (its gravity moment is pose-dependent and the
  // startup tare only cancels it at the tare pose); whether it also changes
  // this SLOPE depends on how much inertia it adds.
  double contact_moment_speed_gain_ = 0.0;  // Nm per (rad/s)

  // Backstop that does not depend on the wrench estimate at all: if the QP
  // model outruns the real arm by this much, treat it as an obstruction,
  // pause, and resync. Exists because the wrench estimate has now twice
  // failed to notice a real contact, and the consequence is not a missed
  // pause but a permanent deadlock - once divergence passes the bridge's
  // model_real_gate (0.05 rad) publishing stops and nothing recovers it.
  // Set at 60% of that gate so this fires first, with the gate left intact
  // as the last line of defence. 0 disables.
  double divergence_pause_ = 0.03;  // rad
  const char * pause_cause_ = "";   // which test latched the current pause

  // What to do if the real arm still hasn't reached the target
  // settle_timeout_ seconds past schedule. false (default) = HOLD and keep
  // logging loudly; true = advance anyway. See the long note at the
  // decision site in run() for why the default flipped on 2026-09-07.
  bool advance_on_timeout_ = false;

  // RETURN-TO-HOLD-POINT (2026-10-07). On release, drive the arm back to the
  // pose it was heading for when contact began, THEN restart the clock -
  // instead of rewinding the clock to wherever the push left the arm.
  // What the person sees: arm stops -> they move it -> they let go -> it
  // returns to where it was -> it carries on. The rewind alternative
  // silently accepts the displacement and resumes from the new spot, which
  // quietly abandons part of the original path.
  // Set false to restore the rewind behaviour (see the note at the resume
  // edge in run()).
  bool   return_to_hold_point_ = true;
  bool   returning_   = false;   // in the return phase (clock still frozen)
  double return_time_ = 0.0;     // seconds spent returning, for the timeout

  // Saved posture-task gains, restored in teardown() - see the back-off in
  // start() and the root-cause note there.
  double prev_posture_weight_    = 1.0;
  double prev_posture_stiffness_ = 1.0;

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
    if(config.has("contact_moment_threshold")) contact_moment_threshold_ = config("contact_moment_threshold");
    if(config.has("contact_moment_speed_gain")) contact_moment_speed_gain_ = config("contact_moment_speed_gain");
    if(config.has("divergence_pause"))        divergence_pause_        = config("divergence_pause");
    if(config.has("advance_on_timeout"))      advance_on_timeout_      = config("advance_on_timeout");
    if(config.has("return_to_hold_point"))    return_to_hold_point_    = config("return_to_hold_point");
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
    returning_   = false;
    return_time_ = 0.0;
    tick_        = 0;

    resyncControlToReal(ctl);
    sva::PTransformd final_target = resolveTarget(ctl, target_cfg_, ee_frame_);
    // A trial profile may MOVE this leg's endpoint (see `targets:` in the
    // YAML). Preferred over `waypoints` for a distance manipulation: a
    // via-point makes targetAt()'s per-segment quintic stop the arm dead at
    // that point, so bowing the path also changes motion smoothness. Moving
    // the endpoint keeps the leg single-segment and continuous.
    //
    // Orientation: a 3-value `targets:` entry moves the position and keeps
    // the captured rotation (right for a pure distance manipulation along a
    // line). A 6-value entry replaces the rotation too, which a condition
    // that is a genuinely different ARM POSE needs - far1's tool orientation
    // differs from near's by 7.87 deg at step_2, and inheriting near's there
    // would command a pose that was never captured.
    if(const Eigen::Vector3d * tr = ppc(ctl).translationFor(name()))
    {
      mc_rtc::log::info("[{}] trial '{}' moves endpoint {:.4f} m -> [{:+.4f}, {:+.4f}, {:+.4f}]",
                        name(), ppc(ctl).activeTrial(),
                        (*tr - final_target.translation()).norm(),
                        (*tr)(0), (*tr)(1), (*tr)(2));
      Eigen::Matrix3d R = final_target.rotation();
      if(const Eigen::Vector3d * rp = ppc(ctl).rotationFor(name()))
      {
        // Same composition as poseFromConfig(): E = Rz(yaw)Ry(pitch)Rx(roll)
        const Eigen::Matrix3d R_new =
          (Eigen::AngleAxisd((*rp)(2), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd((*rp)(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd((*rp)(0), Eigen::Vector3d::UnitX())).toRotationMatrix();
        mc_rtc::log::info("[{}]   and rotation by {:.2f} deg -> [{:+.4f}, {:+.4f}, {:+.4f}]",
                          name(),
                          Eigen::AngleAxisd(R_new * R.transpose()).angle() * 180.0 / M_PI,
                          (*rp)(0), (*rp)(1), (*rp)(2));
        R = R_new;
      }
      else
      {
        mc_rtc::log::info("[{}]   rotation kept as captured", name());
      }
      final_target = sva::PTransformd(R, *tr);
    }
    sva::PTransformd start_pose   = ctl.robot().frame(ee_frame_).position();

    // Build the waypoint chain: start -> intermediate waypoints -> final
    // target. Intermediate waypoints inherit the final target's orientation
    // (same convention as CartesianMove's waypoints).
    // A trial profile may replace this leg's via-points (see `trials:` in the
    // YAML). Falls through to the state's own `waypoints:` when the active
    // trial says nothing about this state, so profiles only need to list the
    // legs they actually change.
    const std::vector<Eigen::Vector3d> * wp_override = ppc(ctl).waypointsFor(name());
    const std::vector<Eigen::Vector3d> & wps_in = wp_override ? *wp_override : pos_waypoints_;
    if(wp_override)
      mc_rtc::log::info("[{}] trial '{}' overrides waypoints: {} via-point(s)",
                        name(), ppc(ctl).activeTrial(), wps_in.size());

    waypts_.clear();
    waypts_.push_back(start_pose);
    for(const auto & wp : wps_in)
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
    // GLOBAL SPEED KNOB (2026-09-23): `speed_scale` in the YAML, one number
    // per trial condition instead of editing v_max on every state. Note it
    // divides `duration` as well as multiplying v_max - `duration` is a FLOOR,
    // so a leg pinned by it (MoveUpFromPick is, at 15s vs its 14s v_max term)
    // would otherwise ignore the scale entirely and the cycle would not
    // actually speed up. Clamped to [0.1, 3.0] by the controller.
    const double sc_ = ppc(ctl).speedScale();
    effective_duration_ = std::max({duration_ / sc_,
                                     1.875 * total_chord / std::max(v_max_lin_ * sc_, 1e-6),
                                     1.875 * ori_delta / std::max(v_max_ang_ * sc_, 1e-6)});

    seg_duration_.assign(chord.size(), effective_duration_ / std::max<size_t>(1, chord.size()));
    if(total_chord > 1e-6)
      for(size_t i = 0; i < chord.size(); ++i)
        seg_duration_[i] = effective_duration_ * (chord[i] / total_chord);

    // ROOT CAUSE FIX 2026-09-07 - back off the posture task, exactly as
    // CartesianMove has always done.
    //
    // This state never did, and that is why it has NEVER converged: the FSM
    // posture task stays active with its target left at whatever the
    // preceding JointMove drove it to (e.g. MoveToSafe's joint
    // configuration) and, at the default weight, competes on equal terms
    // with the ImpedanceTask below (also weight 100). The QP settles at an
    // equilibrium partway between the two - which is exactly what every
    // failure of this state has looked like: the arm stops ~0.24-0.29 m
    // short of target, perfectly static, with zero measured wrench, no
    // joint anywhere near a limit, and model-vs-real agreement of 0.0001
    // rad (the model is not being asked to move, rather than failing to).
    //
    // Diagnosed 2026-09-07 after ruling out the alternatives on live data:
    // not a joint limit (joint_3 at 1.16 rad, 1.41 rad of margin), not a
    // singularity (the stuck pose is BETTER conditioned than the healthy
    // start pose - condition number 26.9 vs 75.3, manipulability 0.014 vs
    // 0.0045 - and the remaining direction needs only 0.023 rad/s on
    // joint_3), and not the contact-pause clock (logged as running, force
    // 0.00 N, 45 s past the end of the trajectory).
    //
    // The tell was that CartesianMove converges cleanly on the same robot
    // (ReturnHome reached pos_err 0.0004 m) and differs in exactly this.
    // NOTE this bug was masked for weeks by the old force-advance-on-timeout
    // default: MoveToPick never actually reached its target, it just got
    // close and the FSM moved on regardless.
    if(auto pt = ctl.getPostureTask(ctl.robot().name()))
    {
      prev_posture_weight_    = pt->weight();
      prev_posture_stiffness_ = pt->stiffness();
      pt->weight(1.0);
      pt->stiffness(1.0);
    }

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
        "[{}] Compliant move started - effective duration {:.2f}s (min {:.2f}s), "
        "v_max_lin={:.3f} m/s, v_max_ang={:.3f} rad/s, path_len={:.4f} m, ori_delta={:.4f} rad, "
        "{} waypoint(s) | BINDS ON {} | [speed_scale {:.2f}x; configured lin {:.3f} ang {:.3f} "
        "min {:.2f}s]",
        name(), effective_duration_, duration_ / sc_, v_max_lin_ * sc_, v_max_ang_ * sc_,
        total_chord, ori_delta, wps_in.size(),
        (1.875 * ori_delta / std::max(v_max_ang_ * sc_, 1e-6) >= effective_duration_ - 1e-9)
            ? "v_max_ang"
            : ((1.875 * total_chord / std::max(v_max_lin_ * sc_, 1e-6) >= effective_duration_ - 1e-9)
                   ? "v_max_lin"
                   : "duration"),
        sc_, v_max_lin_, v_max_ang_, duration_);
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
    const double f = task_->measuredWrench().force().norm();
    const double m = task_->measuredWrench().couple().norm();

    // Model-vs-real divergence, same quantity the bridge gates on.
    double dev = 0.0;
    {
      const auto & mq  = ctl.robot().mbc().q;
      const auto & rq  = ctl.realRobot().mbc().q;
      const auto & mbs = ctl.robot().mb().joints();
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        dev = std::max(dev, std::abs(angDiff(mq[ji][0], rq[ji][0])));
      }
    }

    // Real joint speed drives the friction-induced baseline, so the moment
    // threshold rides on it (see contact_moment_speed_gain_ above).
    double max_qd = 0.0;
    {
      const auto & ra  = ctl.realRobot().mbc().alpha;
      const auto & mbs = ctl.realRobot().mb().joints();
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        max_qd = std::max(max_qd, std::abs(ra[ji][0]));
      }
    }
    const double m_thresh = contact_moment_threshold_ + contact_moment_speed_gain_ * max_qd;

    const bool by_force  = f > contact_force_threshold_;
    const bool by_moment = m > m_thresh;
    // divergence_pause_ MUST scale with speed, same reasoning as
    // stall_lag_enter_ in JointMove above. `dev` is the model's lead over the
    // real arm - a following error, so proportional to commanded velocity and
    // therefore to speed_scale. Pinned at the value that suits base (0.010)
    // it sits ON normal operation for the fastest leg at speed_scale 2.5:
    //
    //   condition  max dq   leg @fast  peak dq/dt   expected dev
    //   near       1.0167   10.6 s     0.1795 rad/s   0.0100  <- trips
    //   far1       0.8285   11.3 s     0.1374 rad/s   0.0077
    //   far2       0.6103    8.8 s     0.1304 rad/s   0.0073
    //
    // which is exactly what was seen: near_fast paused with both wrench
    // channels reading zero -
    //   "CLOCK PAUSED - triggered by model-vs-real divergence: force
    //    0.00/2.00 N, moment 0.00/2.00 Nm, model-vs-real 0.0100/0.0100 rad"
    // while far1_fast and far2_fast ran clean. Near's compliant leg is ~30%
    // faster than the others, so it was the only condition reaching the flat
    // threshold - the bug was there for all three, only near exposed it.
    //
    // TRADE-OFF, deliberate: scaling keeps the false-trigger margin constant
    // (~2.5x) at every speed, but a human must hold the arm back 2.5x further
    // to trip THIS path at fast than at slow. Not tuning - at fast the normal
    // following error is already as large as a light hold, so the two are not
    // separable on this signal. The force and moment channels still work at
    // fast (attenuated ~48% by the bridge's velocity gate). A spurious pause
    // mid-trial corrupts data worse than a missed light touch.
    //
    // NOTE this interacts with delta_max: lowering delta_max lets the model
    // run further ahead, which RAISES dev and eats this margin. The two were
    // changed together once (delta_max 0.006 -> 0.004 alongside this scaling)
    // and the effects cancelled, producing more pauses rather than fewer.
    // Change one at a time.
    const double dev_thresh = divergence_pause_ * ppc(ctl).speedScale();
    const bool by_dev    = divergence_pause_ > 0.0 && dev > dev_thresh
                           && !ppc(ctl).dryRunValidation();

    const bool was_paused = paused_;
    if(by_force || by_moment || by_dev)
    {
      paused_      = true;
      clear_timer_ = 0.0;
      if(by_force)       pause_cause_ = "force";
      else if(by_moment) pause_cause_ = "moment";
      else               pause_cause_ = "model-vs-real divergence";
    }
    else if(paused_)
    {
      clear_timer_ += dt_;
      if(clear_timer_ >= clear_hold_time_) paused_ = false;
    }

    // CLOCK DIAGNOSTIC (2026-09-07): edge-triggered, so it prints once per
    // pause and once per resume rather than every tick. Added after a live
    // HoverToPlace test where the arm was held, stopped, resumed on release,
    // but then never reconverged and FORCED ADVANCE fired mid-traverse -
    // the question being whether the trajectory clock genuinely froze and
    // restarted, or whether something else stalled it. t_elapsed_ is the
    // clock itself, so comparing its value across the pause/resume pair
    // answers that directly: it must be identical at PAUSE and RESUME.
    if(paused_ != was_paused)
    {
      if(paused_)
      {
        mc_rtc::log::warning(
            "[{}] CLOCK PAUSED  - triggered by {}: force {:.2f}/{:.2f} N, moment {:.2f}/{:.2f} Nm, "
            "model-vs-real {:.4f}/{:.4f} rad. t_elapsed frozen at {:.2f}s of {:.2f}s",
            name(), pause_cause_, f, contact_force_threshold_, m, m_thresh,
            dev, dev_thresh, t_elapsed_, effective_duration_);
      }
      else
      {
        // CLOCK REWIND ON RESUME (2026-09-23).
        //
        // While paused we glue the model to the encoders, but the trajectory
        // clock keeps its old time parameterisation - so targetAt(t_elapsed_)
        // sits AHEAD of where the arm actually is. On resume the impedance
        // task closes that gap at whatever its gains allow, with no v_max
        // bound (the quintic shapes the original path, never a catch-up).
        // Live, that burst blew past delta_max's tracking limit within 0.18s
        // and re-tripped the divergence backstop, giving a ~1 Hz
        // brake-and-go limit cycle: resume -> sprint -> diverge -> pause ->
        // snap back -> resume, visible as the arm juddering along.
        //
        // Rewinding the clock to the point on the path nearest the arm's
        // actual pose removes the gap entirely, so the resumed motion starts
        // from zero position error and is speed-bounded by the same quintic
        // as everything else. The PATH is untouched (waypoints included) -
        // only the time cursor moves - and it can only ever move BACKWARD,
        // so a resume can never skip part of the trajectory.
        if(return_to_hold_point_)
        {
          // Contact cleared: do NOT restart the clock yet. Enter the return
          // phase - the trajectory target stays frozen at targetAt(t_elapsed_),
          // which is exactly the pose the arm was heading for when the push
          // started, and the impedance task pulls the arm back to it. The
          // clock restarts only once the arm is actually back (see below).
          // The return speed is not shaped by the quintic, but it IS bounded
          // by the bridge's delta_max clamp, and the displacement is only as
          // large as the person made it - so this is not the unbounded
          // catch-up that caused the 2026-09-23 judder, which came from the
          // target having advanced far downstream while the arm stood still.
          returning_   = true;
          return_time_ = 0.0;
          const double back = (ctl.robot().frame(ee_frame_).position().translation()
                               - targetAt(t_elapsed_).translation()).norm();
          mc_rtc::log::warning(
              "[{}] CONTACT CLEARED - returning {:.3f} m to the hold point before resuming. "
              "Clock stays frozen at {:.2f}s of {:.2f}s",
              name(), back, t_elapsed_, effective_duration_);
        }
        else
        {
          // REWIND alternative (2026-09-23): accept the displacement and
          // resume from the nearest point on the path. Cheaper and smoother,
          // but it abandons the part of the path between the hold point and
          // wherever the push left the arm.
          const double t_before = t_elapsed_;
          {
            const Eigen::Vector3d cur = ctl.robot().frame(ee_frame_).position().translation();
            const int N = 400;
            double best_t = t_elapsed_, best_d = std::numeric_limits<double>::max();
            for(int i = 0; i <= N; ++i)
            {
              const double t = effective_duration_ * static_cast<double>(i) / static_cast<double>(N);
              const double dd = (targetAt(t).translation() - cur).norm();
              if(dd < best_d) { best_d = dd; best_t = t; }
            }
            t_elapsed_ = std::min(t_elapsed_, best_t);
          }
          mc_rtc::log::warning(
              "[{}] CLOCK RESUMED - clear for {:.2f}s (force {:.2f} N, moment {:.2f} Nm, "
              "model-vs-real {:.4f} rad). t_elapsed resuming from {:.2f}s of {:.2f}s "
              "[rewound {:.2f}s to match the arm's actual pose]",
              name(), clear_hold_time_, f, m, dev, t_elapsed_, effective_duration_,
              t_before - t_elapsed_);
        }
      }
    }

    // DEADLOCK FIX 2026-09-07: while a human is holding the arm, glue the
    // QP's internal model to the real arm.
    //
    // Pausing the clock freezes the trajectory TARGET, but it never stopped
    // the control robot from converging to that frozen target. ctl.robot()
    // integrates open-loop and (before this) was resynced only at state
    // start - so during a hold the model walked to the target while the
    // real arm was physically restrained, model-vs-real divergence grew
    // past the bridge's model_real_gate (0.05 rad), the bridge refused to
    // publish, and because nothing resyncs mid-run the arm could then NEVER
    // catch up. Live HoverToPlace test: model joint_1 = 0.9713 rad vs real
    // 0.2171 rad, gate tripped, permanent deadlock - the state kept
    // reporting "HOLDING" while the bridge had already stopped publishing
    // for an unrelated reason.
    //
    // Resyncing every tick while paused keeps the model on top of reality,
    // so the divergence never accumulates: the published command stays
    // within a tick of the encoders (the bridge's delta_max clamp then has
    // nothing to fight), and on release the model resumes from where the
    // arm actually is rather than from a fiction.
    if(paused_) { resyncControlToReal(ctl); returning_ = false; }

    // RETURN PHASE (2026-10-07). Contact has cleared but the clock is still
    // frozen: the target sits at targetAt(t_elapsed_) - the pose the arm was
    // heading for when the push began - and the impedance task pulls the arm
    // back to it. Deliberately NOT resyncing here: during the pause the
    // resync keeps the model on the encoders so divergence cannot build, but
    // during the return the task must be allowed to drive the model away
    // from the (displaced) arm, otherwise it would simply sit where the
    // person left it.
    // Judged against realRobot(): the point is that the ARM comes back, not
    // the model.
    if(returning_ && !paused_)
    {
      return_time_ += dt_;
      const double back = (ctl.realRobot().frame(ee_frame_).position().translation()
                           - targetAt(t_elapsed_).translation()).norm();
      if(back < pos_threshold_)
      {
        returning_ = false;
        mc_rtc::log::success("[{}] Back at the hold point ({:.4f} m) after {:.1f}s - CLOCK RESUMED "
                             "from {:.2f}s of {:.2f}s", name(), back, return_time_,
                             t_elapsed_, effective_duration_);
      }
      else if(return_time_ > settle_timeout_)
      {
        // Could not get back - the arm is still obstructed, or the person is
        // holding it below the contact threshold. Degrade to the rewind
        // rather than hanging: accept the displacement and carry on from the
        // nearest point on the path.
        const Eigen::Vector3d cur = ctl.robot().frame(ee_frame_).position().translation();
        double best_t = t_elapsed_, best_d = std::numeric_limits<double>::max();
        for(int i = 0; i <= 400; ++i)
        {
          const double t = effective_duration_ * static_cast<double>(i) / 400.0;
          const double dd = (targetAt(t).translation() - cur).norm();
          if(dd < best_d) { best_d = dd; best_t = t; }
        }
        t_elapsed_ = std::min(t_elapsed_, best_t);
        returning_ = false;
        mc_rtc::log::error("[{}] Could NOT return to the hold point ({:.4f} m still off after "
                           "{:.1f}s) - falling back to rewinding the clock to {:.2f}s and "
                           "resuming. The arm may still be obstructed.",
                           name(), back, return_time_, t_elapsed_);
      }
    }

    if(!paused_ && !returning_) t_elapsed_ += dt_;

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
    // DRY-RUN VALIDATION (2026-09-30): judge convergence against the MODEL
    // rather than the encoders. In dry_run nothing is published, so
    // realRobot() can never reach the target and the state would hold
    // forever - only one leg per run could be inspected. Comparing the model
    // lets the whole chain walk through in RViz. Live, this is always
    // realRobot(): the only thing that proves the ARM got there.
    const auto & conv_robot = ppc(ctl).dryRunValidation() ? ctl.robot() : ctl.realRobot();
    auto cur = conv_robot.frame(ee_frame_).position();
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
      // Clock + contact state added 2026-09-07 alongside the edge-triggered
      // CLOCK PAUSED/RESUMED lines above: during a live HoverToPlace stall
      // the only thing this line said was that pos_err was static, which
      // could not distinguish "still paused on contact" from "resumed but
      // not converging". Now it states the clock position, whether the
      // clock is running, and the live force, every time it prints.
      // Escalates to an error past the deadline (mirrors CartesianMove), so
      // an arm that is holding rather than advancing is impossible to miss.
      // Model-vs-real divergence, computed here rather than only in the
      // bridge (2026-09-07). The bridge's model_real_gate blocks publishing
      // once this exceeds its threshold, but the STATE could not see that -
      // during a live deadlock this line kept reporting "HOLDING, will not
      // advance" while the real reason nothing moved was that the bridge had
      // silently stopped publishing 28 s earlier. Surfacing it here makes
      // the two failure modes distinguishable in one log line.
      double maxdev = 0.0;
      std::string devjoint;
      {
        const auto & mq  = ctl.robot().mbc().q;
        const auto & rq  = ctl.realRobot().mbc().q;
        const auto & mbs = ctl.robot().mb().joints();
        for(size_t ji = 0; ji < mbs.size(); ++ji)
        {
          if(mbs[ji].dof() != 1) continue;
          double d = std::abs(angDiff(mq[ji][0], rq[ji][0]));
          if(d > maxdev) { maxdev = d; devjoint = mbs[ji].name(); }
        }
      }

      const bool past_deadline = t_elapsed_ > effective_duration_ + settle_timeout_;
      if(past_deadline && !advance_on_timeout_)
        mc_rtc::log::error(
            "[{}] NOT converged {:.2f}s past schedule (pos_err={:.4f} m, ori_err={:.4f} rad) - "
            "HOLDING here, will NOT advance to {} until the real arm reaches the target. "
            "clock {:.2f}/{:.2f}s{} | force {:.2f} N | model-vs-real {:.4f} rad on '{}'{}",
            name(), t_elapsed_ - effective_duration_ - settle_timeout_, pos_err, ori_err,
            next_state_, t_elapsed_, effective_duration_,
            (paused_ ? " PAUSED - contact" : (returning_ ? " RETURNING" : " running")),
            f, maxdev, devjoint,
            ppc(ctl).dryRunValidation()
                ? "  (dry_run_validation: model-vs-real is meaningless here - nothing is published)"
                : (maxdev > 0.05 ? "  <<< EXCEEDS model_real_gate - bridge is NOT publishing, arm"
                                   " cannot recover on its own; restart the state/controller" : ""));
      else
        mc_rtc::log::warning(
            "[{}] Settling: pos_err={:.4f} m, ori_err={:.4f} rad | clock {:.2f}/{:.2f}s{} | "
            "force {:.2f}/{:.2f} N, moment {:.2f}/{:.2f} Nm",
            name(), pos_err, ori_err, t_elapsed_, effective_duration_,
            (paused_ ? " PAUSED - contact" : (returning_ ? " RETURNING to hold point" : " running")),
            f, contact_force_threshold_, m, m_thresh);

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
      // DEFAULT CHANGED 2026-09-07: advance -> hold.
      //
      // This used to force-advance unconditionally (added 2026-08-24) on the
      // reasoning that a stuck compliant move shouldn't hang an experiment
      // session mid-trial. A live HoverToPlace test showed the cost of that:
      // the operator held the arm, released it, it failed to reconverge, and
      // this timeout advanced the FSM into UnloadObj - which tipped the
      // carried container 0.77 rad short of the unload site, i.e. dumped the
      // payload in the wrong place, mid-traverse. "Don't hang the session"
      // is not worth an irreversible physical action at an unintended
      // location, and CartesianMove already made exactly this change on
      // 2026-08-19 (it holds and escalates its log instead) - the two are
      // now consistent.
      //
      // Set `advance_on_timeout: true` per-state to opt back in where the
      // next state is harmless if the target was missed. Do NOT set it where
      // the next state grips, releases, tips, or otherwise commits to
      // something physical.
      // dry_run_validation also advances: the whole point of that mode is to
      // walk the entire chain in RViz, and nothing is published so a
      // not-quite-converged leg commits nothing. Holding here would mean
      // validating one leg per run.
      if(advance_on_timeout_ || ppc(ctl).dryRunValidation())
      {
        mc_rtc::log::error(
            "[{}] FORCED ADVANCE -> {}: NOT converged after {:.2f}s extra "
            "(pos_err={:.4f} m, ori_err={:.4f} rad) - target may not have been "
            "reached; treat this trial/cycle as suspect.",
            name(), next_state_, settle_timeout_, pos_err, ori_err);
        output(next_state_);
        return true;
      }
      // Holding: the periodic log above escalates to the "NOT converged ...
      // holding here" error, so this is loud rather than silent.
    }

    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    if(task_) ctl.solver().removeTask(task_);

    // Restore the posture task gains backed off in start().
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
  std::map<std::string, std::vector<double>> target_joints_yaml_;  // un-wrapped baseline
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
  // ADDED 2026-10-21: `use_speed_scale: false` makes this state ignore the
  // trial's speed_scale and always run at its own v_max/duration. For legs
  // whose speed is a throughput choice rather than part of the experimental
  // manipulation - the unload wrist roll - and which at speed_scale 2.5
  // would otherwise ask for more than the ~0.17 rad/s a joint can track
  // through the bridge (delta_max 0.006 / ~34 ms response). Both uses of
  // the scale below (duration and stall threshold) go through scale(), so
  // they can never disagree. Default true: every other state is unchanged.
  bool use_speed_scale_ = true;
  double scale(mc_control::fsm::Controller & ctl) const
  {
    return use_speed_scale_ ? ppc(ctl).speedScale() : 1.0;
  }
  // ADDED 2026-10-21: `accel_fraction: b` (0 < b <= 0.5) swaps the quintic
  // for a smooth trapezoid - speed ramps up over the first b of the move,
  // CRUISES at peak, and ramps down over the last b. The quintic spends the
  // whole move accelerating or decelerating and touches its peak for an
  // instant (peak/average 1.875); the cruise profile holds the peak, so at
  // the SAME peak velocity - which is what the bridge limits - the move is
  // shorter by a factor 1/(1.875 (1 - b)): 37% at b = 0.15. The ramps follow
  // the quintic smoothstep, so acceleration and jerk are still zero at both
  // ends. For legs that are long purely because they are slow at the
  // bridge's ceiling - the unload wrist roll. Default 0: the quintic,
  // unchanged, for every other state.
  double accel_fraction_ = 0.0;
  double shapeS(double u) const
  {
    if(accel_fraction_ <= 0.0) return quinticS(u);
    const double b = accel_fraction_;
    u = std::clamp(u, 0.0, 1.0);
    // integral of the smoothstep 10x^3 - 15x^4 + 6x^5 from 0 to x
    auto I = [](double x) { return x * x * x * x * (2.5 - 3.0 * x + x * x); };
    if(u <= b)       return b * I(u / b) / (1.0 - b);
    if(u <= 1.0 - b) return (0.5 * b + (u - b)) / (1.0 - b);
    return 1.0 - b * I((1.0 - u) / b) / (1.0 - b);
  }
  // peak / average velocity of the active profile
  double peakRatio() const { return accel_fraction_ > 0.0 ? 1.0 / (1.0 - accel_fraction_) : 1.875; }
  // ADDED 2026-09-07: if every joint is already within this many rad of
  // `target` when the state starts, skip the move entirely and advance
  // immediately instead of holding a no-op reference for the full
  // effective_duration_. Motivating case: the pick-and-place loop's first
  // step is "check if at Zero, go there if not" - without this, starting a
  // cycle from Zero burns the full `duration` (15s by default) doing
  // literally nothing, since convergence is only ever tested AFTER
  // t_elapsed_ >= effective_duration_ (see run()). 0 = disabled, which is
  // the default, so every state that doesn't opt in keeps its old behavior
  // exactly. The startup pose-capture logs still print before the skip
  // decision, so the "jog the arm, read the log" workflow used to derive
  // every pose in the YAML is unaffected.
  double skip_if_within_ = 0.0;

  // ADDED 2026-09-23: same wrench-independent backstop ComplianceCartesianMove
  // got on 2026-09-18, ported here after a live UnloadObj failure.
  //
  // JointMove is rigid by design - no compliance, no contact detection - but
  // that only governs how it RESPONDS to an obstruction, not what happens to
  // the model when the real arm physically cannot follow. UnloadObj commands
  // joint_5 from 0.8323 to 0.0143 rad (a 46.9 deg wrist tip to empty the
  // container); live, the real joint stalled dead at 0.4095 rad - 52% through
  // - while the model marched on to 0.2688. Divergence passed the bridge's
  // model_real_gate (0.05 rad), publishing stopped, and nothing recovers from
  // that without a restart. Every other joint had reached target, so the
  // reported err=0.3952 was exactly |0.4095 - 0.0143|, i.e. joint_5 alone.
  //
  // Freezing the interpolation clock and resyncing the model to the encoders
  // keeps divergence at roughly one tick's worth, so the gate never latches:
  // a blocked joint now HOLDS loudly instead of deadlocking the session.
  // 0 disables.
  double divergence_pause_ = 0.03;  // rad
  std::string next_state_;

  double t_elapsed_           = 0.0;
  double dt_                  = 0.01; //0.005;
  double prev_weight_         = 1.0;
  double prev_stiffness_      = 1.0;
  double effective_duration_  = 3.0;  // = max(duration_, time needed so peak velocity <= v_max_)
  bool   skipped_             = false;
  bool   stalled_             = false;
  double stall_time_          = 0.0;
  // Stall-guard thresholds on the REAL arm's distance from the reference
  // (see the note in run()). <= 0 means "derive from divergence_pause_",
  // resolved in configure().
  //
  // These must straddle the STEADY-STATE following error, which is not
  // small: measured live at speed_scale 1.0 the arm sits 0.035-0.041 rad
  // behind its reference while tracking perfectly well (100%+ by the
  // bridge's own VEL diagnostic). Entry therefore cannot be anywhere near
  // divergence_pause_ (0.03) or the guard would latch continuously during
  // normal motion.
  double stall_lag_enter_     = -1.0;   // default 3 * divergence_pause_ = 0.09
                                        // SCALED BY speed_scale at use - see run()
  double stall_clear_lag_     = -1.0;   // default divergence_pause_ / 2 = 0.015
  std::map<std::string, double> q_start_;

  int tick_ = 0;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("duration"))  duration_  = config("duration");
    if(config.has("stiffness")) stiffness_ = config("stiffness");
    if(config.has("weight"))    weight_    = config("weight");
    if(config.has("threshold")) threshold_ = config("threshold");
    if(config.has("v_max"))     v_max_     = config("v_max");
    if(config.has("use_speed_scale")) use_speed_scale_ = config("use_speed_scale");
    if(config.has("accel_fraction"))  accel_fraction_ = std::clamp(static_cast<double>(config("accel_fraction")), 0.0, 0.5);
    if(config.has("skip_if_within")) skip_if_within_ = config("skip_if_within");
    if(config.has("divergence_pause")) divergence_pause_ = config("divergence_pause");
    if(config.has("stall_lag_enter")) stall_lag_enter_ = config("stall_lag_enter");
    if(config.has("stall_clear_lag")) stall_clear_lag_ = config("stall_clear_lag");
    // Entry well above the ~0.04 steady-state following error so normal
    // motion never trips it; exit below it, which is reachable only because
    // the reference is frozen while latched and the arm can actually close
    // the gap. A genuinely blocked joint crosses 0.09 within ~2 s at the
    // 0.05 rad/s these legs run at.
    if(stall_lag_enter_ <= 0.0) stall_lag_enter_ = 3.0 * divergence_pause_;
    if(stall_clear_lag_ <= 0.0) stall_clear_lag_ = divergence_pause_ / 2.0;
    if(config.has("next"))      next_state_ = static_cast<std::string>(config("next"));

    // target: [v1, v2, ... v6]  (array form only — simplest and most common)
    if(config.has("target"))
    {
      std::vector<double> vals = config("target");
      for(size_t i = 0; i < vals.size(); ++i)
      {
        target_joints_["joint_" + std::to_string(i + 1)] = {vals[i]};
      }
      // Baseline kept separately: start() WRAPS target_joints_ in place to the
      // nearest equivalent angle, and a trial profile may replace it
      // entirely. Without this, a wrap or an override from one trial would
      // leak into the next.
      target_joints_yaml_ = target_joints_;
    }
  }

  // Same quintic (minimum-jerk) time-scaling as ComplianceCartesianMove
  // above - 10u^3-15u^4+6u^5, zero velocity/acceleration at both ends.
  static double quinticS(double u)
  {
    u = std::clamp(u, 0.0, 1.0);
    return 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u;
  }

void start(mc_control::fsm::Controller & ctl) override
  {
    dt_        = ctl.solver().dt();
    t_elapsed_ = 0.0;
    tick_      = 0;

    resyncControlToReal(ctl);
    skipped_    = false;
    stalled_    = false;
    stall_time_ = 0.0;

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

    // ── Trial joint-target override ──────────────────────────────────
    // Restore the YAML baseline first, then let the active trial replace
    // it. Must happen HERE, in start(), not in configure(): the profile can
    // change between trials, and the wrap below rewrites target_joints_ in
    // place so a one-shot assignment would leak the previous trial's wrap
    // into the next one.
    //
    // BUG 2026-10-14: this block existed but sat inside a COMMENTED-OUT
    // duplicate of start() further up this file, so trial `joint_targets:`
    // had never taken effect for any JointMove state. Caught when far1's
    // start_1 drove to near's pose: the log showed
    //     'joint_1' current=1.6528  target=3.2520
    // i.e. near's start_1 (-3.0312 + 2*pi), with neither the override line
    // nor the "Our target_joints_" line that this block prints.
    if(!target_joints_yaml_.empty()) target_joints_ = target_joints_yaml_;
    if(const std::vector<double> * jt = ppc(ctl).jointTargetFor(name()))
    {
      for(size_t i = 0; i < jt->size(); ++i)
        target_joints_["joint_" + std::to_string(i + 1)] = {(*jt)[i]};
      mc_rtc::log::info("[{}] trial '{}' overrides joint target", name(),
                        ppc(ctl).activeTrial());
    }

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

      // Continuous joints only - see isContinuous(). A limited joint keeps
      // its YAML target and goes there the way its limits allow.
      if(!isContinuous(jname)) continue;
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
    // GLOBAL SPEED KNOB (2026-09-23): `speed_scale` in the YAML, one number
    // per trial condition instead of editing v_max on every state. Note it
    // divides `duration` as well as multiplying v_max - `duration` is a FLOOR,
    // so a leg pinned by it (MoveUpFromPick is, at 15s vs its 14s v_max term)
    // would otherwise ignore the scale entirely and the cycle would not
    // actually speed up. Clamped to [0.1, 3.0] by the controller.
    const double sc_ = scale(ctl);
    effective_duration_ = std::max(duration_ / sc_,
                                   peakRatio() * max_abs_delta / std::max(v_max_ * sc_, 1e-6));

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
        "[{}] Joint-space move started - effective duration {:.2f}s (min {:.2f}s), "
        "v_max={:.4f} rad/s, max |delta|={:.4f} rad -> peak velocity {:.4f} rad/s "
        "[speed_scale {:.2f}x{}; configured v_max {:.4f}, min {:.2f}s]",
        name(), effective_duration_, duration_ / sc_, v_max_ * sc_, max_abs_delta,
        peakRatio() * max_abs_delta / effective_duration_, sc_,
        use_speed_scale_ ? "" : " FIXED - trial speed ignored", v_max_, duration_);
    if(accel_fraction_ > 0.0)
      mc_rtc::log::info("[{}] cruise profile: ramps over the first and last {:.0f}% ({:.2f}s each), "
                        "holds the peak in between", name(), 100.0 * accel_fraction_,
                        accel_fraction_ * effective_duration_);

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

    // "Already there?" check (see skip_if_within_ above). Deliberately placed
    // at the very END of start(), after the logs: the gains have already been
    // set and the initial (s=0, "stay put") target written, so teardown()
    // restores them exactly as in the normal path - and the pose-capture logs
    // above still print either way.
    if(skip_if_within_ > 0.0 && max_abs_delta < skip_if_within_)
    {
      skipped_ = true;
      mc_rtc::log::success(
          "[{}] Already at target (max |delta|={:.4f} rad < skip_if_within={:.4f} rad) - skipping move.",
          name(), max_abs_delta, skip_if_within_);
    }
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    if(skipped_) { output(next_state_); return true; }

    auto pt = ctl.getPostureTask(ctl.robot().name());
    if(!pt) { output(next_state_); return true; }

    // Stall guard (see divergence_pause_ above). Advance the interpolation
    // clock ONLY while the real arm is still keeping up; if it has fallen
    // behind, freeze the reference and glue the model to the encoders so the
    // gap cannot accumulate into a model_real_gate deadlock.
    double dev = 0.0;
    std::string devjoint;
    {
      const auto & mq  = ctl.robot().mbc().q;
      const auto & rq  = ctl.realRobot().mbc().q;
      const auto & mbs = ctl.robot().mb().joints();
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        double d = std::abs(angDiff(mq[ji][0], rq[ji][0]));
        if(d > dev) { dev = d; devjoint = mbs[ji].name(); }
      }
    }
    // Interpolated reference at a given clock value.
    auto refAt = [&](double t) {
      double u = effective_duration_ > 1e-6 ? t / effective_duration_ : 1.0;
      double sq = shapeS(u);
      const auto & mq = ctl.robot().mbc().q;
      std::map<std::string, std::vector<double>> r;
      for(const auto & kv : target_joints_)
      {
        double start = q_start_.count(kv.first) ? q_start_.at(kv.first) : kv.second[0];
        double v = (1.0 - sq) * start + sq * kv.second[0];

        // Re-express the reference in the branch the MODEL is currently on.
        //
        // start() unwraps target_joints_ so the interpolation takes the short
        // way round, which for a continuous joint routinely puts the
        // reference outside [-pi, pi] (start_1 -> step_1 interpolates
        // joint_1 from -2.9995 down to -4.683). The model's own q, however,
        // IS wrapped. The instant the joint crosses the branch cut the
        // posture task therefore sees reference -3.20 against model +3.08 -
        // a 2*pi error - and commands an enormous velocity to "correct" it.
        //
        // Live, 2026-10-12, speed_scale 1.0 (so nowhere near any tracking
        // ceiling - the arm was at 104% tracking one tick earlier):
        //     15.80  model=-3.1353            arm-vs-ref 0.0368
        //     STALL  model=+3.0844 real=+3.1012
        //            VEL cmd=-0.7298 real=-0.2239  (0.05 rad/s feedforward!)
        // The angDiff() added earlier fixed the DIAGNOSTICS (dev, lag, err)
        // but not this - the control path still fed an unwrapped reference.
        //
        // Wrapping per-sample keeps the reference within pi of the model at
        // all times. It stays smooth: when the model's q jumps by 2*pi at the
        // cut, so does the reference, and their difference never does.
        if(isContinuous(kv.first) && ctl.robot().hasJoint(kv.first))
        {
          auto idx = ctl.robot().jointIndexByName(kv.first);
          if(static_cast<size_t>(idx) < mq.size() && mq[idx].size() == 1)
          {
            const double cur = mq[idx][0];
            while(v - cur >  M_PI) v -= 2.0 * M_PI;
            while(v - cur < -M_PI) v += 2.0 * M_PI;
          }
        }
        r[kv.first] = {v};
      }
      return r;
    };

    // How far the REAL arm is behind the reference the clock currently holds.
    //
    // THIS - not `dev` - is the exit criterion (2026-10-12). While stalled we
    // resync the model onto the encoders every tick, which forces `dev` to ~0
    // BY CONSTRUCTION. Testing `dev` therefore un-latched the guard on the
    // very next tick, and the lag that had accumulated hit the posture task
    // as a step input. Live on start_1 that gave a ~70 ms brake-and-go
    // sawtooth - stalls at 17.50/17.56/17.63/17.71/17.78/17.85/17.92 s, with
    // commanded velocity spiking to 0.7950 rad/s against 0.0308 real (4%
    // tracking) on each re-entry. Measuring the arm against the frozen
    // reference is immune to the resync, so the guard stays latched until the
    // arm has genuinely caught up.
    double lag = 0.0;
    {
      const auto ref   = refAt(t_elapsed_);
      const auto & rq  = ctl.realRobot().mbc().q;
      const auto & mbs = ctl.robot().mb().joints();
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        auto it = ref.find(mbs[ji].name());
        if(it == ref.end()) continue;
        lag = std::max(lag, std::abs(angDiff(rq[ji][0], it->second[0])));
      }
    }

    // LATCH CRITERION (reworked 2026-10-12, second pass).
    //
    // The guard means "the real arm is not following". That is `lag` - how
    // far the ARM is from the reference - NOT `dev`, which is how far the
    // MODEL is from the arm. Those differ whenever the model overshoots its
    // own reference, which it routinely does for a tick or two around the
    // +/-pi crossing, and gating on `dev` alone made every trip so far a
    // false positive:
    //     trip         dev       lag
    //     run A      0.0301    0.0034
    //     run B      0.0303    0.0074
    //                0.0307    0.0015
    //                0.0303    0.0031
    //                0.0306    0.0078
    //                0.0301    0.0121
    // The arm was never more than 12 mrad behind - it was tracking at
    // 100%+. Each false trip then resynced, which steps the model by ~0.03
    // and is itself a disturbance: commanded velocity jumped 0.0255 ->
    // 0.1590 rad/s, the model overshot the reference by 0.035, and the task
    // dragged it back (err climbing 0.0478 -> 0.0576). Brake, lurch, drift -
    // precisely the roughness reported near start_1.
    //
    // So: latch on `lag`. `dev` is kept only as a runaway backstop, at 1.5x
    // the threshold - still under the bridge's model_real_gate (0.05), past
    // which it stops publishing and needs a restart.
    const bool can_stall = divergence_pause_ > 0.0 && !ppc(ctl).dryRunValidation();
    // stall_lag_enter_ MUST scale with speed: the lag it is testing is the
    // posture task's steady-state following error, which is proportional to
    // the reference velocity and therefore to speed_scale. Measured live:
    //     speed_scale 1.0 -> reference 0.05  rad/s -> lag 0.035-0.041
    //     speed_scale 2.5 -> reference 0.125 rad/s -> lag 0.090-0.100
    // a ratio of 2.25 against a speed ratio of 2.5. Pinned at the 1.0 value
    // (0.09) the threshold sits BELOW normal operation at speed_scale 2.5,
    // so the guard tripped every time the quintic reached peak velocity
    // mid-leg and froze the clock for ~2 s each time - felt as the speed
    // repeatedly dropping and recovering. Scaling keeps the margin at a
    // constant ~2.2x at every speed.
    // NOTE this is the ARM-vs-reference branch only; the model-runaway
    // backstop below stays absolute, because the bridge's model_real_gate
    // it protects against is itself absolute.
    const double lag_enter = stall_lag_enter_ * scale(ctl);
    const bool arm_behind = lag > lag_enter;
    const bool model_runaway = dev > 1.5 * divergence_pause_;
    if(!stalled_ && can_stall && (arm_behind || model_runaway))
    {
      stalled_    = true;
      stall_time_ = 0.0;
      // Resync ONCE, here, and never again while latched. This collapses the
      // lead the model built up before the guard fired (at entry the model is
      // typically ahead of its OWN reference too - 0.0303 model-vs-real
      // against 0.0074 arm-vs-ref in the 2026-10-12 live run).
      //
      // Doing it every tick deadlocks the arm, which is exactly what the
      // first version of this guard did: the published command is derived
      // from the model's q, so pinning the model to the encoder every tick
      // caps the commanded lead at one tick of QP motion (~1e-4 rad), far too
      // little to break stiction. The log showed model=real=3.0750 frozen,
      // arm-vs-ref stuck at 0.0154 against a 0.0100 exit threshold, and
      // VEL cmd=-0.0125 with real=0.0000 - commanded forever, never moving.
      //
      // With the clock frozen the reference is stationary, so skipping the
      // resync cannot let the model run away: it converges to that fixed
      // point and stops, leaving model-vs-real bounded by the entry lag
      // (~0.03), comfortably under the bridge's model_real_gate of 0.05.
      resyncControlToReal(ctl);
      mc_rtc::log::error(
          "[{}] STALLED ({}): arm is {:.4f} rad behind the reference, model-vs-real {:.4f} rad "
          "on '{}' (thresholds {:.4f} / {:.4f}). Freezing the trajectory clock at {:.2f}s of "
          "{:.2f}s and holding the reference there until the arm is back within {:.4f} rad of "
          "it. Will NOT advance to {} until it moves.",
          name(), arm_behind ? "arm behind" : "model runaway", lag, dev, devjoint,
          lag_enter, 1.5 * divergence_pause_, t_elapsed_, effective_duration_,
          stall_clear_lag_, next_state_);
    }

    if(stalled_)
    {
      // NO resync here - see the note at the latch above.
      stall_time_ += dt_;
      if(lag < stall_clear_lag_)
      {
        mc_rtc::log::success(
            "[{}] Stall cleared after {:.2f}s (arm now {:.4f} rad from the reference) - "
            "resuming from {:.2f}s.", name(), stall_time_, lag, t_elapsed_);
        stalled_    = false;
        stall_time_ = 0.0;
      }
      else if(stall_time_ > 5.0 && std::fmod(stall_time_, 5.0) < dt_)
      {
        mc_rtc::log::error(
            "[{}] STILL stalled after {:.0f}s - arm {:.4f} rad behind the reference "
            "(needs < {:.4f}). Physically blocked, or the target is unreachable.",
            name(), stall_time_, lag, stall_clear_lag_);
      }
    }
    else
    {
      t_elapsed_ += dt_;
    }

    // Feed the interpolated (slow, bounded) reference every tick - the
    // posture task's stiffness_/weight_ now only controls how tightly it
    // tracks THIS moving reference, not how fast the motion itself is.
    pt->target(refAt(t_elapsed_));

    // Convergence judged against the REAL arm (realRobot()), not
    // pt->eval() (the posture task's own ctl.robot()-internal error) -
    // same reasoning as CartesianMove/ComplianceCartesianMove above.
    // EXCEPT under dry_run_validation, where nothing is published so
    // realRobot() can never move and this would never converge - there the
    // model is compared instead so the chain walks through in RViz.
    double err = 0.0;
    {
      const auto & q   = (ppc(ctl).dryRunValidation() ? ctl.robot() : ctl.realRobot()).mbc().q;
      const auto & mbs = ctl.robot().mb().joints();
      for(size_t ji = 0; ji < mbs.size(); ++ji)
      {
        if(mbs[ji].dof() != 1) continue;
        const std::string & jname = mbs[ji].name();
        if(!target_joints_.count(jname)) continue;
        double d = angDiff(q[ji][0], target_joints_.at(jname)[0]);
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
      mc_rtc::log::info("[{}] err={:.4f} | clock {:.2f}/{:.2f}s{} | model-vs-real {:.4f} rad "
                        "on '{}' | arm-vs-ref {:.4f} rad",
                        name(), err, t_elapsed_, effective_duration_,
                        stalled_ ? " STALLED - frozen" : " running", dev, devjoint, lag);
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
  // Optional explicit knuckle-joint position (rad) overriding the action
  // preset - see sendGripperGoal() in PickPlaceController.h for the scale.
  // Negative = not specified = use the action preset (previous behavior).
  double      position_  = -1.0;
  std::string next_state_;

  bool   sent_      = false;
  double t_elapsed_ = 0.0;
  double dt_        = 0.005;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("action"))  action_  = static_cast<std::string>(config("action"));
    if(config.has("timeout")) timeout_ = config("timeout");
    if(config.has("position")) position_ = config("position");
    if(config.has("next"))    next_state_ = static_cast<std::string>(config("next"));
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    dt_        = ctl.solver().dt();
    t_elapsed_ = 0.0;
    ppc(ctl).resetGripperDone();

    // We do NOT send the goal on start() because the DDS discovery might not be ready.
    sent_ = false;
    if(position_ >= 0.0)
    {
      mc_rtc::log::info("[{}] Gripper state initialized. Waiting to establish connection for action: {} "
                        "(explicit position {:.3f} rad)", name(), action_, position_);
    }
    else
    {
      mc_rtc::log::info("[{}] Gripper state initialized. Waiting to establish connection for action: {}", name(), action_);
    }
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    t_elapsed_ += dt_;

    // Attempt to send the goal to ROS 2 until the action server is discovered and ready
    if(!sent_)
    {
      sent_ = ppc(ctl).sendGripperGoal(action_, position_);
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
// TRIAL GATE (2026-09-30, gate moved after MoveToSafe on the same day).
// Idle used to be a permanent dead end. It is now where the controller parks
// BEFORE each trial, immediately after MoveToSafe: on launch the arm goes to
// the safe pose and waits rather than running a cycle unprompted, and at the
// end of every task it returns there and waits again. It holds position
// until a trial name arrives on /trial_config, applies that profile
// (speed_scale + waypoint overrides), and releases into MoveToPick.
//
// Applying the profile HERE, on the control thread and between cycles, is
// deliberate: changing speed mid-leg would discontinuously retime a
// trajectory already in flight, and every motion state reads speedScale()
// in its own start(), so a change made here is picked up cleanly by every
// leg of the next cycle.
//
// Nothing restarts between trials - driver, bridge, wrench tare and the
// arm's live state all persist, which is the whole point.
struct Idle : mc_control::fsm::State
{
  std::string next_state_;
  bool announced_ = false;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("next")) next_state_ = static_cast<std::string>(config("next"));
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    announced_ = false;
    // Open the gate: trial requests are accepted ONLY while parked here.
    // Anything published mid-cycle is rejected with a warning rather than
    // latched - a couple of seconds of `ros2 topic pub` (no --once) used to
    // queue a second request and silently run the whole cycle twice.
    ppc(ctl).setAtGate(true);
    mc_rtc::log::success("[TrialGate] Parked at the safe pose (last trial: '{}'). Waiting for a "
                         "trial on /trial_config - the arm holds here and nothing needs "
                         "restarting between trials.",
                         ppc(ctl).activeTrial());
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    if(next_state_.empty()) return false;          // no trial gate configured
    if(ppc(ctl).consumePendingTrial())
    {
      // Route to the trial's own chain if it names one, else fall through to
      // this state's `next:`. The transition table must list a row for every
      // entry state used, keyed on that same output name.
      const std::string entry = ppc(ctl).entryFor();
      const std::string & dest = entry.empty() ? next_state_ : entry;
      if(!entry.empty())
        mc_rtc::log::info("[TrialGate] trial '{}' -> entering its own chain at '{}'",
                          ppc(ctl).activeTrial(), dest);
      output(dest);
      return true;
    }
    if(!announced_)
    {
      announced_ = true;
      mc_rtc::log::info("[TrialGate] waiting - publish a trial name to /trial_config to run one");
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override { ppc(ctl).setAtGate(false); }
};

// ════════════════════════════════════════════════════════════════════════════
//  DirectJointMove — a joint move executed by the JTC, not streamed (2026-10-21)
// ════════════════════════════════════════════════════════════════════════════
// Every other motion state STREAMS: mc_rtc recomputes the reference each
// tick and the bridge sends the joint_trajectory_controller one point every
// 10 ms, never more than delta_max (0.006 rad) ahead of the MEASURED
// position. The arm answers each command ~34 ms late, so that leash caps any
// joint near 0.006 / 0.034 = 0.17 rad/s - on purpose: before it existed the
// model ran away from the arm three times. Right for the compliant legs,
// which must keep reacting; wrong for a large, pre-planned move nobody
// interacts with, like the 170 deg unload wrist roll (~26 s even cruising).
//
// This state does what a conventional controller does: it plans the whole
// move up front and sends it ONCE, as a single timed JointTrajectory with
// position/velocity/acceleration on every point, and the JTC plays it back
// at its own rate. The limit becomes the actuator (URDF: 1.22 rad/s for
// joint_6), not the leash.
//
// Handshake with the bridge, through the controller datastore:
//   KortexBridge::HoldStreaming  (set here)  the bridge publishes nothing while
//                                            true - each of its single-point
//                                            messages would REPLACE this
//                                            trajectory in the JTC
//   KortexBridge::StreamingHeld  (bridge)    acknowledgement; nothing is sent
//                                            before it reads true
//   KortexBridge::DryRun         (bridge)    its dry_run - this state never moves
//                                            the arm when the bridge would not
// If the bridge does not publish those keys (impedance_control not rebuilt)
// the state REFUSES and holds: it never sends a trajectory the bridge would
// fight. Under dry_run (bridge or dry_run_validation) it animates the model
// along the same plan instead, so the chain still walks through in RViz.
//
// While the JTC drives, the model is glued to the encoders every tick, so
// when streaming resumes the bridge's first command is the arm's own
// position - no jump. That per-tick resync is safe HERE only because the
// bridge is not publishing (in JointMove it capped the commanded lead and
// deadlocked the arm). mc_rtc's stall guard is not running, so this state
// has its own: if the real arm falls more than track_tolerance off the plan
// - blocked, or the Kinova faulted - it stops the JTC where the arm is,
// hands control back to the bridge, and holds without advancing.
//
// Speed: FIXED by default (speed_scale ignored - the unload wrist roll).
// `use_speed_scale: true` makes v_max and min_duration follow the trial's
// speed_scale exactly as JointMove's do (the base-rotation legs, 2026-10-21).
// Profile: `cruise` (default) is a smooth trapezoid - ramp over the first
// `accel_fraction`, cruise at v_max, ramp down - with zero acceleration and
// jerk at both ends; `quintic` is JointMove's own curve, so a leg moved here
// from JointMove keeps exactly the timing it had.
//
// Continuous joints may cross +/-pi. The plan is sent as ONE continuous
// angle (e.g. joint_1 -3.03 -> -4.54, the short way to +1.74) rather than
// wrapped, because the JTC interpolates between consecutive points as given
// and a wrapped point would jump 2*pi mid-trajectory. The driver treats
// angles 2*pi apart as the same position - the streamed legs cross the cut
// every cycle with commands that do exactly that - and every check here is
// wrap-aware, so if the arm ever went the other way the track_tolerance
// abort would stop it within 0.15 rad.
struct DirectJointMove : mc_control::fsm::State
{
  static constexpr const char * kHold   = "KortexBridge::HoldStreaming";
  static constexpr const char * kHeld   = "KortexBridge::StreamingHeld";
  static constexpr const char * kDryRun = "KortexBridge::DryRun";

  // Config
  std::vector<double> target_;          // joint_1..joint_6
  double v_max_          = 0.5;   // rad/s, peak velocity of the joint that moves furthest
  double accel_fraction_ = 0.25;  // share of the move spent ramping up (and again down)
  double min_duration_   = 1.0;   // s
  double threshold_      = 0.03;  // rad, REAL arm within this of target -> done
  double track_tol_      = 0.15;  // rad, REAL arm this far off the plan -> abort
  double settle_timeout_ = 5.0;   // s after the plan ends before complaining
  double sample_dt_      = 0.05;  // s between points sent to the JTC
  bool   use_speed_scale_ = false; // true: v_max and min_duration follow speed_scale
  bool   quintic_         = false; // profile: quintic -> JointMove's exact timing
  double skip_if_within_  = -1.0;  // rad; already this close on every joint -> skip
  double max_joint_move_  = -1.0;  // rad; any joint further than this -> refuse (<= 0 off)
  std::string next_state_;

  // Runtime
  enum class Phase { Refused, Skipped, Simulating, WaitHold, Running, Settling, Aborted };
  Phase phase_ = Phase::Refused;
  std::vector<double> q0_, dq_;   // start, and signed shortest-path delta
  double T_ = 0.0, t_ = 0.0, wait_ = 0.0, settle_ = 0.0, dt_ = 0.001;
  double next_log_ = 0.0, next_complaint_ = 0.0;

  void configure(const mc_rtc::Configuration & config) override
  {
    if(config.has("target"))
    {
      std::vector<double> v = config("target");
      target_ = v;
    }
    if(config.has("v_max"))           v_max_          = config("v_max");
    if(config.has("accel_fraction"))  accel_fraction_ = config("accel_fraction");
    if(config.has("min_duration"))    min_duration_   = config("min_duration");
    if(config.has("threshold"))       threshold_      = config("threshold");
    if(config.has("track_tolerance")) track_tol_      = config("track_tolerance");
    if(config.has("settle_timeout"))  settle_timeout_ = config("settle_timeout");
    if(config.has("next"))            next_state_     = static_cast<std::string>(config("next"));
    if(config.has("use_speed_scale")) use_speed_scale_ = config("use_speed_scale");
    if(config.has("skip_if_within"))  skip_if_within_  = config("skip_if_within");
    if(config.has("max_joint_move"))  max_joint_move_  = config("max_joint_move");
    if(config.has("profile"))
    {
      const std::string prof = static_cast<std::string>(config("profile"));
      if(prof == "quintic")     quintic_ = true;
      else if(prof == "cruise") quintic_ = false;
      else mc_rtc::log::error("[DirectJointMove] unknown profile '{}' - using cruise", prof);
    }
    accel_fraction_ = std::clamp(accel_fraction_, 0.05, 0.5);
    // Sanity bound, not a tuning value: 82% of the URDF limit for the small
    // actuators (1.2218 rad/s, joints 4-6).
    if(v_max_ > 1.0 || v_max_ <= 0.0)
    {
      mc_rtc::log::error("[DirectJointMove] v_max {:.3f} outside (0, 1.0] rad/s - clamped", v_max_);
      v_max_ = std::clamp(v_max_, 0.01, 1.0);
    }
  }

  // s, ds/du, d2s/du2 on u in [0,1]. Quintic: JointMove's 10u^3-15u^4+6u^5.
  // Otherwise the smooth trapezoid, same shape as JointMove's accel_fraction.
  static void profile(double u, double b, bool quintic, double & s, double & ds, double & dds)
  {
    u = std::clamp(u, 0.0, 1.0);
    if(quintic)
    {
      s   = u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
      ds  = 30.0 * u * u * (1.0 - u) * (1.0 - u);
      dds = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u);
      return;
    }
    auto I  = [](double x) { return x * x * x * x * (2.5 - 3.0 * x + x * x); };   // integral of S
    auto S  = [](double x) { return x * x * x * (10.0 - 15.0 * x + 6.0 * x * x); }; // smoothstep
    auto dS = [](double x) { return 30.0 * x * x * (1.0 - x) * (1.0 - x); };         // its slope
    const double k = 1.0 / (1.0 - b);
    if(u <= b)
    {
      const double x = u / b;
      s = b * I(x) * k; ds = S(x) * k; dds = dS(x) * k / b;
    }
    else if(u <= 1.0 - b)
    {
      s = (0.5 * b + (u - b)) * k; ds = k; dds = 0.0;
    }
    else
    {
      const double y = (1.0 - u) / b;
      s = 1.0 - b * I(y) * k; ds = S(y) * k; dds = -dS(y) * k / b;
    }
  }

  static std::string jointName(size_t k) { return "joint_" + std::to_string(k + 1); }

  std::vector<double> planAt(double t) const
  {
    double s, ds, dds;
    profile(T_ > 1e-9 ? t / T_ : 1.0, accel_fraction_, quintic_, s, ds, dds);
    std::vector<double> q(6);
    for(size_t k = 0; k < 6; ++k) q[k] = q0_[k] + dq_[k] * s;
    return q;
  }

  // Max |a_k - b_k| over the six joints, wrap-aware.
  double maxErr(const mc_rbdyn::Robot & robot, const std::vector<double> & ref) const
  {
    double e = 0.0;
    for(size_t k = 0; k < 6; ++k)
    {
      const auto idx = robot.jointIndexByName(jointName(k));
      e = std::max(e, std::abs(angDiff(robot.mbc().q[idx][0], ref[k])));
    }
    return e;
  }

  void setPosture(mc_control::fsm::Controller & ctl, const std::vector<double> & q)
  {
    if(auto pt = ctl.getPostureTask(ctl.robot().name()))
    {
      std::map<std::string, std::vector<double>> tgt;
      for(size_t k = 0; k < 6; ++k) tgt[jointName(k)] = {q[k]};
      pt->target(tgt);
    }
  }

  // Model := real arm, and the posture task holds it there.
  void glue(mc_control::fsm::Controller & ctl)
  {
    resyncControlToReal(ctl);
    std::vector<double> q(6);
    for(size_t k = 0; k < 6; ++k)
      q[k] = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointName(k))][0];
    setPosture(ctl, q);
  }

  std::vector<double> target() const
  {
    std::vector<double> q(6);
    for(size_t k = 0; k < 6; ++k) q[k] = q0_[k] + dq_[k];
    return q;
  }

  bool sendPlan(mc_control::fsm::Controller & ctl)
  {
    const int n = std::max(1, static_cast<int>(std::ceil(T_ / sample_dt_)));
    std::vector<double> times;
    std::vector<std::vector<double>> Q, QD, QDD;
    times.reserve(n); Q.reserve(n); QD.reserve(n); QDD.reserve(n);
    for(int i = 1; i <= n; ++i)
    {
      const double t = T_ * static_cast<double>(i) / n;
      double s, ds, dds;
      profile(t / T_, accel_fraction_, quintic_, s, ds, dds);
      std::vector<double> q(6), qd(6), qdd(6);
      for(size_t k = 0; k < 6; ++k)
      {
        q[k]   = q0_[k] + dq_[k] * s;
        qd[k]  = dq_[k] * ds / T_;
        qdd[k] = dq_[k] * dds / (T_ * T_);
      }
      times.push_back(t); Q.push_back(q); QD.push_back(qd); QDD.push_back(qdd);
    }
    return ppc(ctl).sendJointTrajectory(times, Q, QD, QDD);
  }

  // Stop the JTC where the arm IS and give control back to the bridge,
  // which then streams the model - glued to the arm - i.e. holds.
  void abort(mc_control::fsm::Controller & ctl, const char * why, double err)
  {
    glue(ctl);
    std::vector<double> q(6);
    for(size_t k = 0; k < 6; ++k)
      q[k] = ctl.robot().mbc().q[ctl.robot().jointIndexByName(jointName(k))][0];
    const std::vector<double> zero(6, 0.0);
    ppc(ctl).sendJointTrajectory({0.2}, {q}, {zero}, {zero});
    ctl.datastore().get<bool>(kHold) = false;
    phase_ = Phase::Aborted;
    next_complaint_ = 5.0;
    mc_rtc::log::error("[{}] ABORTED at {:.2f}s of {:.2f}s: {} ({:.4f} rad, tolerance {:.4f}). "
                       "Stopped the arm where it is and handed back to the bridge. Will NOT "
                       "advance to {} - check the arm, then restart the trial.",
                       name(), t_, T_, why, err, track_tol_, next_state_);
  }

  void start(mc_control::fsm::Controller & ctl) override
  {
    dt_ = ctl.solver().dt();
    t_ = wait_ = settle_ = 0.0;
    next_log_ = 0.0;
    next_complaint_ = 5.0;
    auto & ds = ctl.datastore();
    if(!ds.has(kHold)) ds.make<bool>(kHold, false);

    // Plan from the REAL arm, the shortest way round for continuous joints.
    // The WHOLE current pose is captured before any check can fail: every
    // refusal path below holds the posture task at q0_, so a partially
    // filled q0_ would hold the unfilled joints at ZERO - and the bridge
    // would stream the arm there. dq_ stays zero unless the plan is valid.
    resyncControlToReal(ctl);
    q0_.assign(6, 0.0);
    dq_.assign(6, 0.0);
    for(size_t k = 0; k < 6; ++k)
      q0_[k] = angDiff(ctl.realRobot().mbc().q[ctl.realRobot().jointIndexByName(jointName(k))][0], 0.0);
    setPosture(ctl, q0_);
    bool ok = target_.size() == 6;
    if(!ok)
      mc_rtc::log::error("[{}] needs `target` with 6 joint values, got {}", name(), target_.size());
    double max_abs = 0.0;
    mc_rtc::log::info("[{}] Joint current_q vs target (after wrap):", name());
    for(size_t k = 0; ok && k < 6; ++k)
    {
      const std::string jn = jointName(k);
      const auto idx = ctl.robot().jointIndexByName(jn);
      if(isContinuous(jn))
        dq_[k] = angDiff(target_[k], q0_[k]);   // shortest way round
      else
      {
        // Limited joint: the real target, and it must be inside the limits.
        dq_[k] = target_[k] - q0_[k];
        const double lo = ctl.robot().ql()[idx][0], hi = ctl.robot().qu()[idx][0];
        if(target_[k] < lo || target_[k] > hi)
        {
          mc_rtc::log::error("[{}] REFUSED: '{}' target {:.4f} is outside its limits [{:.4f}, {:.4f}]",
                             name(), jn, target_[k], lo, hi);
          ok = false;
        }
      }
      mc_rtc::log::info("[{}]   '{}' current={:.4f}  target={:.4f}  delta={:.4f}",
                        name(), jn, q0_[k], q0_[k] + dq_[k], dq_[k]);
      // Crossing +/-pi is allowed: sent as one continuous angle - see the
      // note above the struct.
      if(std::abs(q0_[k] + dq_[k]) > M_PI)
        mc_rtc::log::info("[{}]   '{}' crosses +/-pi: sent as one continuous angle {:.4f} -> {:.4f} "
                          "(= {:.4f})", name(), jn, q0_[k], q0_[k] + dq_[k], angDiff(q0_[k] + dq_[k], 0.0));
      max_abs = std::max(max_abs, std::abs(dq_[k]));
    }
    // Entry legs (start_*) run from wherever the arm was left. If that is far
    // from the pose - after maintenance, a web-app jog, a different elbow
    // configuration - a straight joint-space move can sweep through the
    // workspace or into the arm's own body. Refuse, and say where to jog to.
    if(ok && max_joint_move_ > 0.0 && max_abs > max_joint_move_)
    {
      mc_rtc::log::error("[{}] REFUSED: the arm is {:.2f} rad from this pose on at least one joint "
                         "(max_joint_move {:.2f}). Jog it close to [{:.4f}, {:.4f}, {:.4f}, {:.4f}, "
                         "{:.4f}, {:.4f}] rad with the web app, then restart the trial.",
                         name(), max_abs, max_joint_move_, target_[0], target_[1], target_[2],
                         target_[3], target_[4], target_[5]);
      ok = false;
    }
    const double sc = use_speed_scale_ ? ppc(ctl).speedScale() : 1.0;
    double v_eff = v_max_ * sc;
    if(v_eff > 1.0)
    {
      mc_rtc::log::error("[{}] v_max {:.3f} x speed_scale {:.2f} = {:.3f} rad/s - capped at 1.0",
                         name(), v_max_, sc, v_eff);
      v_eff = 1.0;
    }
    const double peak_ratio = quintic_ ? 1.875 : 1.0 / (1.0 - accel_fraction_);
    T_ = std::max(min_duration_ / sc, peak_ratio * max_abs / v_eff);

    if(!ok)
    {
      dq_.assign(6, 0.0);
      phase_ = Phase::Refused;
      return;
    }
    if(skip_if_within_ > 0.0 && max_abs <= skip_if_within_)
    {
      phase_ = Phase::Skipped;
      mc_rtc::log::success("[{}] already within {:.4f} rad of target (max |delta| {:.4f}) - skipping",
                           name(), skip_if_within_, max_abs);
      return;
    }
    mc_rtc::log::info("[{}] DIRECT JTC move: max |delta| {:.4f} rad in {:.2f}s - peak {:.3f} rad/s, {} profile{}. "
                      "speed_scale {:.2f}{}.",
                      name(), max_abs, T_, peak_ratio * max_abs / T_, quintic_ ? "quintic" : "cruise",
                      quintic_ ? "" : " (ramps over the first and last " +
                                      std::to_string(static_cast<int>(std::lround(100.0 * accel_fraction_))) + "%)",
                      sc, use_speed_scale_ ? "" : " (ignored - fixed speed)");

    if(ppc(ctl).dryRunValidation() || (ds.has(kDryRun) && ds.get<bool>(kDryRun)))
    {
      phase_ = Phase::Simulating;
      mc_rtc::log::warning("[{}] dry run - NOT sending to the JTC; animating the model along the "
                           "same plan instead", name());
      return;
    }
    if(!ds.has(kHeld) || !ds.has(kDryRun))
    {
      phase_ = Phase::Refused;
      mc_rtc::log::error("[{}] REFUSED: the bridge does not support the streaming hold (no '{}' / "
                         "'{}' in the datastore). Rebuild impedance_control - the bridge lives "
                         "there. Holding here; nothing was sent.", name(), kHeld, kDryRun);
      return;
    }
    ds.get<bool>(kHold) = true;
    phase_ = Phase::WaitHold;
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    auto & ds = ctl.datastore();
    switch(phase_)
    {
      case Phase::Refused:
      case Phase::Aborted:
        settle_ += dt_;
        if(settle_ >= next_complaint_)
        {
          next_complaint_ += 5.0;
          mc_rtc::log::error("[{}] still holding after {}. Restart the trial once resolved.",
                             name(), phase_ == Phase::Refused ? "a refusal" : "an abort");
        }
        return false;

      case Phase::Skipped:
        output(next_state_);
        return true;

      case Phase::Simulating:
      {
        t_ += dt_;
        setPosture(ctl, planAt(std::min(t_, T_)));
        if(t_ >= T_ && (maxErr(ctl.robot(), target()) < threshold_ || t_ > T_ + settle_timeout_))
        {
          mc_rtc::log::success("[{}] dry run complete ({:.2f}s) -> {}", name(), t_, next_state_);
          output(next_state_);
          return true;
        }
        return false;
      }

      case Phase::WaitHold:
        glue(ctl);
        wait_ += dt_;
        if(ds.get<bool>(kHeld))
        {
          if(!sendPlan(ctl))
          {
            ds.get<bool>(kHold) = false;
            phase_ = Phase::Refused;
            mc_rtc::log::error("[{}] REFUSED: no ROS publisher for the JTC (stub mode?). "
                               "Holding here; nothing was sent.", name());
            return false;
          }
          mc_rtc::log::info("[{}] bridge streaming held after {:.0f} ms - trajectory sent to the JTC",
                            name(), 1000.0 * wait_);
          t_ = 0.0;
          phase_ = Phase::Running;
        }
        else if(wait_ > 0.5)
        {
          ds.get<bool>(kHold) = false;
          phase_ = Phase::Refused;
          mc_rtc::log::error("[{}] REFUSED: the bridge never acknowledged the streaming hold "
                             "(0.5 s). Holding here; nothing was sent.", name());
        }
        return false;

      case Phase::Running:
      {
        glue(ctl);
        t_ += dt_;
        const double err = maxErr(ctl.realRobot(), planAt(std::min(t_, T_)));
        if(err > track_tol_)
        {
          abort(ctl, "the arm fell off the plan", err);
          return false;
        }
        if(t_ >= next_log_)
        {
          next_log_ += 0.5;
          mc_rtc::log::info("[{}] {:.2f}/{:.2f}s | arm-vs-plan {:.4f} rad", name(), t_, T_, err);
        }
        if(t_ >= T_) phase_ = Phase::Settling;
        return false;
      }

      case Phase::Settling:
      {
        glue(ctl);
        t_ += dt_;
        settle_ += dt_;
        const double err = maxErr(ctl.realRobot(), target());
        if(err < threshold_)
        {
          ds.get<bool>(kHold) = false;   // model == arm, so the bridge resumes with no jump
          mc_rtc::log::success("[{}] done in {:.2f}s (planned {:.2f}s), {:.4f} rad from target -> {}",
                               name(), t_, T_, err, next_state_);
          output(next_state_);
          return true;
        }
        if(err > track_tol_)
        {
          abort(ctl, "the arm stopped short of the target", err);
          return false;
        }
        if(settle_ >= next_complaint_)
        {
          next_complaint_ += 5.0;
          mc_rtc::log::error("[{}] {:.0f}s past the plan and still {:.4f} rad from target "
                             "(needs < {:.4f})", name(), settle_, err, threshold_);
        }
        return false;
      }
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    // Never leave the bridge muted, whatever path led out of this state.
    if(ctl.datastore().has(kHold)) ctl.datastore().get<bool>(kHold) = false;
  }
};

// ════════════════════════════════════════════════════════════════════════════
//  State factory — ONE extern "C" block for all state classes.
// ════════════════════════════════════════════════════════════════════════════
extern "C"
{
  __attribute__((visibility("default")))
  void MC_RTC_FSM_STATE(std::vector<std::string> & names)
  {
    names = {"CartesianMove", "ComplianceCartesianMove", "JointMove", "Gripper", "Idle",
             "DirectJointMove"};
  }

  __attribute__((visibility("default")))
  mc_control::fsm::State * create(const std::string & name)
  {
    if(name == "CartesianMove")           return new CartesianMove();
    if(name == "ComplianceCartesianMove") return new ComplianceCartesianMove();
    if(name == "JointMove")               return new JointMove();
    if(name == "Gripper")                 return new Gripper();
    if(name == "Idle")                    return new Idle();
    if(name == "DirectJointMove")         return new DirectJointMove();
    return nullptr;
  }

  __attribute__((visibility("default")))
  void destroy(mc_control::fsm::State * ptr)
  {
    delete ptr;
  }
}

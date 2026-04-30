#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>

#include <SpaceVecAlg/PTransform.h>

#include <atomic>
#include <string>

// ============================================================
//  PickPlaceController
//
//  Thin FSM controller whose entire behaviour is defined by
//  YAML state configurations (see etc/PickPlaceController.yaml).
//
//  It exposes the three reference poses (home/pick/place) read
//  from YAML to the generic state classes (CartesianMove,
//  JointMove, Gripper) via accessor methods.
//
//  Gripper methods are provided as stubs; connect them to your
//  real gripper driver (ROS action client, Kinova Kortex API,
//  vacuum gripper, etc.) in the implementation.
// ============================================================
class PickPlaceController : public mc_control::fsm::Controller
{
public:
  PickPlaceController(mc_rbdyn::RobotModulePtr rm,
                      double dt,
                      const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & d) override;

  // ── Reference-pose accessors (read from YAML at construction) ────────────
  const sva::PTransformd & homePose()  const { return home_pose_;  }
  const sva::PTransformd & pickPose()  const { return pick_pose_;  }
  const sva::PTransformd & placePose() const { return place_pose_; }
  double                   zMinLimit() const { return z_min_limit_; }

  // ── Gripper interface (stubbed; wire this to your real gripper) ──────────
  // Called by the `Gripper` FSM state with action ∈ {"open", "close"}.
  void sendGripperGoal(const std::string & action);
  bool isGripperDone() const;
  void resetGripperDone();

private:
  sva::PTransformd home_pose_;
  sva::PTransformd pick_pose_;
  sva::PTransformd place_pose_;
  double           z_min_limit_ = 0.15;

  std::atomic<bool> gripper_done_{true};
};
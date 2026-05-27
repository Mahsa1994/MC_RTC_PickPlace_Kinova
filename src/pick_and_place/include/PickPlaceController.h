#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>

#include <SpaceVecAlg/PTransform.h>

#include <atomic>
#include <string>

// ROS2
#ifdef MC_RTC_HAS_ROS_SUPPORT
#include <mc_rtc_ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#endif

// ============================================================
//  PickPlaceController
//
//  FSM controller whose entire behaviour is defined by
//  YAML state configurations (etc/PickPlaceController.yaml).
//
//  It exposes the three reference poses (home/pick/place) read
//  from YAML to the generic state classes (CartesianMove,
//  JointMove, Gripper) via accessor methods.
// ============================================================
class PickPlaceController : public mc_control::fsm::Controller
{
public:
  PickPlaceController(mc_rbdyn::RobotModulePtr rm,
                      double dt,
                      const mc_rtc::Configuration &config);

  bool run() override;
  void reset(const mc_control::ControllerResetData &d) override;

  //  Reference-pose accessors (read from YAML at construction)
  const sva::PTransformd &homePose() const { return home_pose_; }
  const sva::PTransformd &pickPose() const { return pick_pose_; }
  const sva::PTransformd &placePose() const { return place_pose_; }
  double zMinLimit() const { return z_min_limit_; }

  //  Gripper interface (stubbed; wire this to your real gripper)
  // Called by the `Gripper` FSM state with action ∈ {"open", "close"}.
  void sendGripperGoal(const std::string &action);
  bool isGripperDone() const;
  void resetGripperDone();

private:
  sva::PTransformd home_pose_;
  sva::PTransformd pick_pose_;
  sva::PTransformd place_pose_;
  double z_min_limit_ = 0.15;

  std::atomic<bool> gripper_done_{true};

#ifdef MC_RTC_HAS_ROS_SUPPORT
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  // Retain a pointer to the mc_rtc global node
  mc_rtc::NodeHandlePtr nh_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
#endif
};

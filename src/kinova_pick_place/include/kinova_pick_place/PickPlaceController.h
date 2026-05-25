// include/kinova_pick_place/PickPlaceController.h
#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace kinova_pick_place {

struct PickPlaceController : public mc_control::fsm::Controller
{
  PickPlaceController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  // Persistent Cartesian task
  std::shared_ptr<mc_tasks::EndEffectorTask> eeTask_;

  // Shared targets
  Eigen::Vector3d pickPosition_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond pickOrientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d placePosition_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond placeOrientation_ = Eigen::Quaterniond::Identity();

  // Parameters
  double approachOffset_ = 0.15;
  double gripperForceThreshold_ = 5.0;
  bool gripperClosed_ = false;
  double dt_ = 0.005;

  // ROS interface (states will need these)
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripperPub_;

  // Helpers used by states
  void sendGripperCommand(double position, double duration_sec = 1.0);
  bool nearTarget(const Eigen::Vector3d & target, double tol = 0.02);

private:
  void setupROS();
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pickSub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr placeSub_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrenchSub_;
};

} // namespace kinova_pick_place

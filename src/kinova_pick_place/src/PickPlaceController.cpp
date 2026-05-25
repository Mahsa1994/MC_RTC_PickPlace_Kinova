// src/PickPlaceController.cpp
#include "kinova_pick_place/PickPlaceController.h"
#include <mc_rtc/logging.h>
#include <sva/PTransformd.h>

namespace kinova_pick_place {

PickPlaceController::PickPlaceController(
  mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config), dt_(dt)
{
  config_("approach_offset", approachOffset_);
  config_("gripper_force_threshold", gripperForceThreshold_);

  if (!hasRobot("gen3")) {
    mc_rtc::log::error_and_throw<std::runtime_error>("No gen3 robot loaded");
  }

  // Always-on end-effector task
  eeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(
    "tool_frame", robots(), 0, 10.0, 5.0);
  eeTask_->addToSolver(solver());

  setupROS();
  mc_rtc::log::success("PickPlaceFSM initialized");
}

void PickPlaceController::setupROS()
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  node_ = std::make_shared<rclcpp::Node>("pick_place_fsm");

  pickSub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
    "/pick_target", 10,
    [this](geometry_msgs::msg::Pose::SharedPtr msg) {
      pickPosition_ << msg->position.x, msg->position.y, msg->position.z;
      pickOrientation_ = Eigen::Quaterniond(
        msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    });

  placeSub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
    "/place_target", 10,
    [this](geometry_msgs::msg::Pose::SharedPtr msg) {
      placePosition_ << msg->position.x, msg->position.y, msg->position.z;
      placeOrientation_ = Eigen::Quaterniond(
        msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    });

  wrenchSub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
    "/gripper/wrench", 10,
    [this](geometry_msgs::msg::Wrench::SharedPtr msg) {
      double f = std::sqrt(
        msg->force.x * msg->force.x +
        msg->force.y * msg->force.y +
        msg->force.z * msg->force.z);
      gripperClosed_ = (f > gripperForceThreshold_);
    });

  gripperPub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/gripper_controller/joint_trajectory", 10);
}

void PickPlaceController::sendGripperCommand(double position, double duration_sec)
{
  if (!gripperPub_) return;
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = {"finger_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = {position};
  pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  traj.points.push_back(pt);
  gripperPub_->publish(traj);
}

bool PickPlaceController::nearTarget(const Eigen::Vector3d & target, double tol)
{
  auto pose = robot("gen3").bodyPosW("tool_frame");
  return (pose.translation() - target).norm() < tol;
}

bool PickPlaceController::run()
{
  rclcpp::spin_some(node_);
  return mc_control::fsm::Controller::run();
}

void PickPlaceController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  eeTask_->reset();
  gripperClosed_ = false;
  pickPosition_.setZero();
  placePosition_.setZero();
}

} // namespace kinova_pick_place

// Controller factory
extern "C" {
  CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)
  {
    names = {"PickPlaceController"};
  }
  CONTROLLER_MODULE_API void destroy_controller(mc_control::MCController * ptr)
  {
    delete ptr;
  }
  CONTROLLER_MODULE_API mc_control::MCController * create_controller(
    const std::string &,
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration & config)
  {
    return new kinova_pick_place::PickPlaceController(rm, dt, config);
  }
}

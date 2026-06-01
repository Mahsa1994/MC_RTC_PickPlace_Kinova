#pragma once
#include <mc_rtc/config.h>
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/PTransform.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <atomic>
#include <string>

// Include ROS 2 integration if supported
#ifdef MC_RTC_HAS_ROS_SUPPORT
//#include <mc_rtc_ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
//#include <control_msgs/action/gripper_command.hpp>
#endif

class PickPlaceController : public mc_control::fsm::Controller
{
public:
  PickPlaceController(mc_rbdyn::RobotModulePtr rm,
                      double dt,
                      const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & d) override;

  // Reference-pose accessors (defined inline)
  const sva::PTransformd & homePose()  const { return home_pose_;  }
  const sva::PTransformd & pickPose()  const { return pick_pose_;  }
  const sva::PTransformd & placePose() const { return place_pose_; }
  double                   zMinLimit() const { return z_min_limit_; }

  // Gripper interface (all fully inline to prevent dynamic linking dependency)
  bool isGripperDone() const { return gripper_done_.load(); }
  void resetGripperDone()     { gripper_done_ = false; }

  bool sendGripperGoal(const std::string & action)
  {
    double target_pos = 0.0;
    if(action == "close")
    {
      target_pos = 0.7; //0.8
    }
    else if(action == "open")
    {
      target_pos = 0.1; //0.0
    }
    else
    {
      mc_rtc::log::error("[Gripper] Unknown gripper action request: {}", action);
      gripper_done_ = true;
      return true;
    }

#ifdef MC_RTC_HAS_ROS_SUPPORT
    if(!nh_ || !gripper_action_client_)
    {
      mc_rtc::log::warning("[Gripper] ROS 2 interface unavailable. Completing instantly as stub.");
      gripper_done_ = true;
      return true;
    }

    if(!gripper_action_client_->action_server_is_ready())
    {
      mc_rtc::log::error("[Gripper] Action server not ready! Cannot actuate gripper.");
      gripper_done_ = true;
      return false;
    }

    // auto goal_msg = control_msgs::action::GripperCommand::Goal();
    // goal_msg.command.position = target_pos;
    gripper_done_ = false;

    auto goal_msg = control_msgs::action::ParallelGripperCommand::Goal();
    goal_msg.command.name     = {"robotiq_85_left_knuckle_joint"};
    goal_msg.command.position = {target_pos};
    goal_msg.command.effort   = {100.0};

    // auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    auto send_goal_options = rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleGripper::SharedPtr & goal_handle) {
        if(!goal_handle)
        {
          mc_rtc::log::error("[Gripper] Goal was rejected by the action server.");
          this->gripper_done_ = true;
        }
        else
        {
          mc_rtc::log::info("[Gripper] Goal accepted, starting execution.");
        }
      };

    send_goal_options.result_callback =
      [this](const GoalHandleGripper::WrappedResult & result) {
        switch(result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            mc_rtc::log::success("[Gripper] Action succeeded.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            mc_rtc::log::error("[Gripper] Action was aborted.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            mc_rtc::log::error("[Gripper] Action was canceled.");
            break;
          default:
            mc_rtc::log::error("[Gripper] Received unknown action result code.");
            break;
        }
        this->gripper_done_ = true;
      };

    gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
#else
    mc_rtc::log::info("[Gripper] Stub mode active. Simulating action: {}", action);
    gripper_done_ = true;
#endif
  }

private:
  sva::PTransformd home_pose_;
  sva::PTransformd pick_pose_;
  sva::PTransformd place_pose_;
  double           z_min_limit_ = 0.15;

  std::atomic<bool> gripper_done_{true};

#ifdef MC_RTC_HAS_ROS_SUPPORT
  // using GripperCommand = control_msgs::action::GripperCommand;
  // using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  mc_rtc::NodeHandlePtr nh_;
  // rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  using GripperCommand    = control_msgs::action::ParallelGripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;


#endif
};

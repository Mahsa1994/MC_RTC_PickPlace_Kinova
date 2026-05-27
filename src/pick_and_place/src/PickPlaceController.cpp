#include "PickPlaceController.h"

#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>

static sva::PTransformd poseFromConfig(const mc_rtc::Configuration &cfg)
{
  Eigen::Vector3d t = cfg("translation");
  Eigen::Vector3d rpy = cfg("rotation");
  Eigen::Matrix3d R =
      (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return sva::PTransformd(R, t);
}

PickPlaceController::PickPlaceController(mc_rbdyn::RobotModulePtr rm,
                                         double dt,
                                         const mc_rtc::Configuration &config)
try : mc_control
  ::fsm::Controller(rm, dt, config)
  {
    home_pose_ = poseFromConfig(config("home_pose"));
    pick_pose_ = poseFromConfig(config("pick_pose"));
    place_pose_ = poseFromConfig(config("place_pose"));
    if (config.has("z_min_limit"))
      z_min_limit_ = config("z_min_limit");

    // Clamp Z of reference poses to the safety floor
    auto clampZ = [&](sva::PTransformd &p)
    {
      Eigen::Vector3d t = p.translation();
      if (t.z() < z_min_limit_)
      {
        t.z() = z_min_limit_;
        p = sva::PTransformd(p.rotation(), t);
      }
    };
    clampZ(pick_pose_);
    clampZ(place_pose_);
    clampZ(home_pose_);

#ifdef MC_RTC_HAS_ROS_SUPPORT
    // Retrieve the global rclcpp::Node from mc_rtc
    nh_ = mc_rtc::ROSBridge::get_node_handle();
    if (nh_)
    {
      gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
          nh_, "/robotiq_gripper_controller/gripper_cmd");
      mc_rtc::log::info("[PickPlaceController] ROS 2 Node handle acquired, Action Client initialized.");
    }
    else
    {
      mc_rtc::log::warning("[PickPlaceController] ROS 2 is not initialized. Using stub mode.");
    }
#endif

    mc_rtc::log::info("[PickPlaceController] Robot: {} (dt={}s)", robot().name(), dt);
    mc_rtc::log::info("[PickPlaceController] home:  [{:+.3f}, {:+.3f}, {:+.3f}]",
                      home_pose_.translation().x(),
                      home_pose_.translation().y(),
                      home_pose_.translation().z());
    mc_rtc::log::info("[PickPlaceController] pick:  [{:+.3f}, {:+.3f}, {:+.3f}]",
                      pick_pose_.translation().x(),
                      pick_pose_.translation().y(),
                      pick_pose_.translation().z());
    mc_rtc::log::info("[PickPlaceController] place: [{:+.3f}, {:+.3f}, {:+.3f}]",
                      place_pose_.translation().x(),
                      place_pose_.translation().y(),
                      place_pose_.translation().z());
    mc_rtc::log::success("[PickPlaceController] Initialized.");
  }
catch (const std::exception &e)
{
  mc_rtc::log::critical("[PickPlaceController] Exception: {}", e.what());
  throw;
}

bool PickPlaceController::run()
{
  return mc_control::fsm::Controller::run();
}

void PickPlaceController::reset(const mc_control::ControllerResetData &d)
{
  mc_control::fsm::Controller::reset(d);
}

//  Gripper
void PickPlaceController::sendGripperGoal(const std::string &action)
{
  double target_pos = 0.0;
  if (action == "close")
  {
    target_pos = 0.7; // Match the physical limit of the Robotiq 85 (0.8 = closed)
  }
  else if (action == "open")
  {
    target_pos = 0.1; // Fully open
  }
  else
  {
    mc_rtc::log::error("[Gripper] Unknown gripper action request: {}", action);
    gripper_done_ = true;
    return;
  }

#ifdef MC_RTC_HAS_ROS_SUPPORT
  if (!nh_ || !gripper_action_client_)
  {
    mc_rtc::log::warning("[Gripper] ROS 2 interface unavailable. Completing instantly as stub.");
    gripper_done_ = true;
    return;
  }

  if (!gripper_action_client_->action_server_is_ready())
  {
    mc_rtc::log::error("[Gripper] Action server not ready! Cannot actuate gripper.");
    gripper_done_ = true;
    return;
  }

  auto goal_msg = control_msgs::action::GripperCommand::Goal();
  goal_msg.command.position = target_pos;

  auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();

  // Callback when server accepts/rejects the goal
  send_goal_options.goal_response_callback =
      [this](const GoalHandleGripper::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      mc_rtc::log::error("[Gripper] Goal was rejected by the action server.");
      this->gripper_done_ = true; // Complete immediately on failure to keep FSM running
    }
    else
    {
      mc_rtc::log::info("[Gripper] Goal accepted, starting execution.");
    }
  };

  // Callback when execution is completed
  send_goal_options.result_callback =
      [this](const GoalHandleGripper::WrappedResult &result)
  {
    switch (result.code)
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
    this->gripper_done_ = true; // Update atomic flag
  };

  gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
#else
  mc_rtc::log::info("[Gripper] Stub mode active. Simulating action: {}", action);
  gripper_done_ = true;
#endif
}

//bool PickPlaceController::isGripperDone() const
//{
//  return gripper_done_.load();
//}

//void PickPlaceController::resetGripperDone()
//{
//  gripper_done_ = false;
//}

CONTROLLER_CONSTRUCTOR("PickPlaceController", PickPlaceController)

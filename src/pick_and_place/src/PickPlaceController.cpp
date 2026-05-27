#include "PickPlaceController.h"

#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>

static sva::PTransformd poseFromConfig(const mc_rtc::Configuration & cfg)
{
  Eigen::Vector3d t   = cfg("translation");
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
                                         const mc_rtc::Configuration & config)
try : mc_control::fsm::Controller(rm, dt, config)
{
  home_pose_   = poseFromConfig(config("home_pose"));
  pick_pose_   = poseFromConfig(config("pick_pose"));
  place_pose_  = poseFromConfig(config("place_pose"));
  if(config.has("z_min_limit")) z_min_limit_ = config("z_min_limit");

  // Clamp Z of reference poses to the safety floor
  auto clampZ = [&](sva::PTransformd & p) {
    Eigen::Vector3d t = p.translation();
    if(t.z() < z_min_limit_) { t.z() = z_min_limit_; p = sva::PTransformd(p.rotation(), t); }
  };
  clampZ(pick_pose_);
  clampZ(place_pose_);
  clampZ(home_pose_);

#ifdef MC_RTC_HAS_ROS_SUPPORT
  // Retrieve the global rclcpp::Node from mc_rtc
  nh_ = mc_rtc::ROSBridge::get_node_handle();
  if(nh_)
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
catch(const std::exception & e)
{
  mc_rtc::log::critical("[PickPlaceController] Exception: {}", e.what());
  throw;
}

bool PickPlaceController::run()
{
  return mc_control::fsm::Controller::run();
}

void PickPlaceController::reset(const mc_control::ControllerResetData & d)
{
  mc_control::fsm::Controller::reset(d);
}

CONTROLLER_CONSTRUCTOR("PickPlaceController", PickPlaceController)

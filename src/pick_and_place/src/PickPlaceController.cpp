#include "PickPlaceController.h"
#include <algorithm>

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

  // GLOBAL SPEED KNOB (2026-09-23) - one number per trial condition instead
  // of editing v_max on eight states. Motion states multiply their v_max*
  // by this and divide `duration` by it; `duration` has to scale too or the
  // duration-floored legs (MoveUpFromPick is one) would simply ignore the
  // change and the cycle would not actually get faster.
  // Clamped hard: every leg currently peaks near 0.05 rad/s of real joint
  // speed and the bridge's delta_max (0.002 rad @ 100 Hz) puts the ceiling
  // at ~0.2 rad/s, i.e. about 4x. 3.0 keeps a margin under that; past it the
  // real arm falls behind the model and the run ends in a STALLED hold.
  if(config.has("speed_scale"))
  {
    speed_scale_ = config("speed_scale");
    if(speed_scale_ < 0.1 || speed_scale_ > 3.0)
    {
      double req = speed_scale_;
      speed_scale_ = std::min(3.0, std::max(0.1, speed_scale_));
      mc_rtc::log::error("[PickPlaceController] speed_scale {:.2f} out of range [0.1, 3.0] - "
                         "clamped to {:.2f}. Above ~4x the real arm cannot track the model "
                         "(delta_max ceiling ~0.2 rad/s).", req, speed_scale_);
    }
  }
  if(speed_scale_ != 1.0)
    mc_rtc::log::warning("[PickPlaceController] speed_scale = {:.2f}x - every motion state's "
                         "v_max* is multiplied and its `duration` divided by this.", speed_scale_);

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
    //gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
    //    nh_, "/robotiq_gripper_controller/gripper_cmd");
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::ParallelGripperCommand>(
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

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

// ═══════════════════════════════════════════════════════════════════════════
//  Gripper stubs
//  ----------------------------------------------------------------------
//  TODO: Wire these to your actual gripper. For the Kinova Gen3, the two
//  common options are:
//    (a) ROS 2 action client sending a GripperCommand goal to Kortex.
//    (b) Direct Kortex API call.
//  The current implementation immediately reports "done" so the FSM
//  flows through CloseGripper/OpenGripper states without real actuation.
// ═══════════════════════════════════════════════════════════════════════════
void PickPlaceController::sendGripperGoal(const std::string & action)
{
  mc_rtc::log::info("[Gripper] Stub received: {} (replace with real driver)", action);
  // For a real implementation:
  //   - Publish to /<robot>/robotiq_gripper_controller/gripper_cmd action
  //   - Or call Kortex SendGripperCommand service
  //   - Set gripper_done_ = true when the action completes
  gripper_done_ = true;  // stub: instant completion
}

bool PickPlaceController::isGripperDone() const
{
  return gripper_done_.load();
}

void PickPlaceController::resetGripperDone()
{
  gripper_done_ = false;
}

CONTROLLER_CONSTRUCTOR("PickPlaceController", PickPlaceController)

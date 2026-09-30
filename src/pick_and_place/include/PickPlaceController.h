#pragma once
#include <mc_rtc/config.h>
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <SpaceVecAlg/PTransform.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <atomic>
#include <string>
#include <map>
#include <mutex>
#include <vector>

// Include ROS 2 integration if supported
#ifdef MC_RTC_HAS_ROS_SUPPORT
//#include <mc_rtc_ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
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

  // Global experiment speed knob (see `speed_scale` in the YAML). Every
  // motion state multiplies its v_max* by this and divides its `duration`
  // by it, so one number makes the whole cycle uniformly faster or slower
  // for a trial condition.
  // NOT const-fixed any more: a trial profile can replace it at runtime (see
  // below). States read it in start(), so a change takes effect from the
  // next leg onward without restarting anything.
  double                   speedScale() const { return speed_scale_; }

  // ── Runtime trial selection (2026-09-30) ────────────────────────────────
  // A "trial" is a named profile in the YAML's `trials:` block overriding
  // speed_scale and per-state waypoints. Selecting one publishes its name on
  // /trial_config; Idle picks it up, applies it, and re-enters the cycle at
  // MoveToSafe. Nothing restarts - the driver, the bridge, the wrench tare
  // and the arm's live state all persist across trials, which is what makes
  // back-to-back experimental runs possible.
  struct TrialProfile
  {
    double speed_scale = 1.0;
    // state name -> via-points for that state's Cartesian leg
    std::map<std::string, std::vector<Eigen::Vector3d>> waypoints;
  };

  /// Waypoint override for `state`, or nullptr if this trial does not set one
  /// (in which case the state keeps whatever its own YAML `waypoints:` says).
  const std::vector<Eigen::Vector3d> * waypointsFor(const std::string & state) const
  {
    auto t = trials_.find(active_trial_);
    if(t == trials_.end()) return nullptr;
    auto w = t->second.waypoints.find(state);
    return w == t->second.waypoints.end() ? nullptr : &w->second;
  }

  const std::string & activeTrial() const { return active_trial_; }

  /// Called from Idle on the control thread. If a trial was requested over
  /// ROS, make it active (applying its speed_scale) and return true.
  /// Applying HERE rather than in the subscriber callback keeps the change
  /// off the ROS thread and guarantees it lands between cycles, never
  /// mid-leg where it would discontinuously retime a trajectory in flight.
  bool consumePendingTrial()
  {
    std::string name;
    {
      std::lock_guard<std::mutex> lock(trial_mutex_);
      if(!trial_pending_) return false;
      name = pending_trial_;
      trial_pending_ = false;
    }
    auto it = trials_.find(name);
    if(it == trials_.end())
    {
      mc_rtc::log::error("[PickPlace] trial '{}' is not defined - staying idle", name);
      return false;
    }
    active_trial_ = name;
    speed_scale_  = it->second.speed_scale;
    mc_rtc::log::success("[PickPlace] TRIAL '{}' starting - speed_scale {:.2f}, {} waypoint override(s)",
                         active_trial_, speed_scale_, it->second.waypoints.size());
    return true;
  }

  // Gripper interface (all fully inline to prevent dynamic linking dependency)
  bool isGripperDone() const { return gripper_done_.load(); }
  void resetGripperDone()     { gripper_done_ = false; }

  // `position` (2026-09-07): optional explicit knuckle-joint target that
  // OVERRIDES the action->position mapping below. Added for the "half open"
  // step of the full pick-and-place loop, where neither of the two hardcoded
  // presets applies. Units are robotiq_85_left_knuckle_joint radians on the
  // same scale the presets already use: ~0.0 = fully open, ~0.8 = fully
  // closed (so open=0.1 / close=0.7 below are near-but-not-at the extremes,
  // and "half open" is ~0.4). Negative means "not specified" -> fall back to
  // the action preset, so every existing `action: open|close` call site keeps
  // its previous behavior untouched.
  bool sendGripperGoal(const std::string & action, double position = -1.0)
  {
    double target_pos = 0.0;
    if(position >= 0.0)
    {
      target_pos = position;
    }
    else if(action == "close")
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
      // mc_rtc::log::error("[Gripper] Action server not ready! Cannot actuate gripper.");
      gripper_done_ = true;
      return false;
    }

    // auto goal_msg = control_msgs::action::GripperCommand::Goal();
    // goal_msg.command.position = target_pos;
    // Logged HERE, not earlier: everything above this point is retried every
    // tick by Gripper::run() until the action server is discovered, so a log
    // before the readiness gate would spam at the control rate.
    mc_rtc::log::info("[Gripper] action '{}' -> knuckle target {:.3f} rad{}",
                      action, target_pos,
                      position >= 0.0 ? " (explicit position override)" : " (action preset)");
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
    return true;
#else
    mc_rtc::log::info("[Gripper] Stub mode active. Simulating action: {} (knuckle target {:.3f} rad)",
                      action, target_pos);
    gripper_done_ = true;
    return true;
#endif
  }

private:
  sva::PTransformd home_pose_;
  sva::PTransformd pick_pose_;
  sva::PTransformd place_pose_;
  double           z_min_limit_ = 0.15;
  double           speed_scale_ = 1.0;

  std::map<std::string, TrialProfile> trials_;
  std::string        active_trial_ = "(none)";
  std::string        pending_trial_;
  bool               trial_pending_ = false;
  mutable std::mutex trial_mutex_;   // guards pending_trial_/trial_pending_

  std::atomic<bool> gripper_done_{true};

#ifdef MC_RTC_HAS_ROS_SUPPORT
  // using GripperCommand = control_msgs::action::GripperCommand;
  // using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  mc_rtc::NodeHandlePtr nh_;
  // rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  using GripperCommand    = control_msgs::action::ParallelGripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_action_client_;
  // OWN node for trial selection (2026-09-30), deliberately NOT mc_rtc's.
  // mc_rtc's ROSBridge node exists to PUBLISH (TF, robot state), which needs
  // no executor - so a subscription created on it is registered and visible
  // to `ros2 topic info` but its callback is never dispatched. That is also
  // the most likely reason the gripper action client on that same node only
  // ever produced "Gripper timeout, proceeding" and never "Goal accepted":
  // action clients need spinning too.
  // This node is spun explicitly from run(), so its callbacks do fire.
  rclcpp::Node::SharedPtr trial_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trial_sub_;
  int trial_spin_tick_ = 0;


#endif
};

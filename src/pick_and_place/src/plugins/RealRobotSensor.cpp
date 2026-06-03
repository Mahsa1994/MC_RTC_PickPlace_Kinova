/**
 * RealRobotSensor — mc_rtc GlobalPlugin
 *
 * Subscribes to /joint_states and feeds real encoder values into mc_rtc
 * before each control tick. This bridges the kortex driver → mc_rtc.
 */
#include <mc_control/GlobalPlugin.h>
//#include <mc_control/MCGlobalController.h>
#include <mc_control/GlobalPluginMacros.h>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mutex>
#include <vector>
#include <string>

class RealRobotSensor : public mc_control::GlobalPlugin
{
public:
  void init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration &) override
  {
    robot_name_ = gc.controller().robot().name();
    const auto & ref_order = gc.controller().robot().refJointOrder();
    joint_names_ = ref_order;
    n_joints_ = joint_names_.size();
    encoder_values_.resize(n_joints_, 0.0);

    // Init ROS2 node
    if(!rclcpp::ok()) { rclcpp::init(0, nullptr); }
    node_ = std::make_shared<rclcpp::Node>("real_robot_sensor_plugin");
    sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          // Build name→index map from message
          for(size_t i = 0; i < joint_names_.size(); ++i)
          {
            for(size_t j = 0; j < msg->name.size(); ++j)
            {
              if(msg->name[j] == joint_names_[i])
              {
                encoder_values_[i] = msg->position[j];
                break;
              }
            }
          }
          received_ = true;
        });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    mc_rtc::log::success("[RealRobotSensor] Plugin initialized, listening to /joint_states");
  }

  void reset(mc_control::MCGlobalController &) override {}

  void before(mc_control::MCGlobalController & gc) override
  {
    if(!received_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    gc.setEncoderValues(robot_name_, encoder_values_);
  }

  void after(mc_control::MCGlobalController &) override {}

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override
  {
    mc_control::GlobalPlugin::GlobalPluginConfiguration cfg;
    cfg.should_run_before = true;
    cfg.should_run_after  = false;
    cfg.should_always_run = true;
    return cfg;
  }

  ~RealRobotSensor() override
  {
    if(executor_) executor_->cancel();
    if(spin_thread_.joinable()) spin_thread_.join();
  }

private:
  std::string robot_name_;
  std::vector<std::string> joint_names_;
  size_t n_joints_{0};
  std::vector<double> encoder_values_;
  std::mutex mutex_;
  bool received_{false};

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::thread spin_thread_;
};

EXPORT_MC_RTC_PLUGIN("RealRobotSensor", RealRobotSensor)

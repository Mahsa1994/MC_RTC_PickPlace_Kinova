#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

class KortexMcRtcBridge : public rclcpp::Node
{
public:
  KortexMcRtcBridge() : Node("kortex_mc_rtc_bridge")
  {
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 1);

    gc_ = std::make_shared<mc_control::MCGlobalController>();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                   .durability(rclcpp::DurabilityPolicy::Volatile);

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos,
        std::bind(&KortexMcRtcBridge::jointStateCallback, this, std::placeholders::_1));

    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/EEForceSensor", 10,
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(wrench_mutex_);
          latest_wrench_ = msg;
          last_wrench_time_ = this->now();
        });

    mc_rtc::log::info("[KortexBridge] Waiting for first /joint_states...");
    mc_rtc::log::info("[KortexBridge] Subscribed to /EEForceSensor");
  }

private:
  std::mutex init_mutex_;
  std::atomic<bool> initialized_{false};

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  geometry_msgs::msg::WrenchStamped::SharedPtr latest_wrench_;
  std::mutex wrench_mutex_;
  rclcpp::Time last_wrench_time_{0, 0, RCL_ROS_TIME};
  bool wrench_active_ = false;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (initialized_)
    {
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> enc_q(ref_order.size(), 0.0);
      std::vector<double> enc_alpha(ref_order.size(), 0.0);
      for (size_t i = 0; i < ref_order.size(); ++i)
        for (size_t j = 0; j < msg->name.size(); ++j)
          if (msg->name[j] == ref_order[i])
          {
            enc_q[i]     = msg->position[j];
            enc_alpha[i] = msg->velocity[j];
            break;
          }
      gc_->setEncoderValues(gc_->robot().name(), enc_q);
      gc_->setEncoderVelocities(gc_->robot().name(), enc_alpha);
      return;
    }

    std::lock_guard<std::mutex> lock(init_mutex_);
    if (initialized_) return;

    RCLCPP_INFO(this->get_logger(), "Seeding mc_rtc with first real joint states...");
    auto ref_order = gc_->robot().refJointOrder();
    std::vector<double> init_q(ref_order.size(), 0.0);
    for (size_t i = 0; i < ref_order.size(); ++i)
      for (size_t j = 0; j < msg->name.size(); ++j)
        if (msg->name[j] == ref_order[i])
        {
          init_q[i] = msg->position[j];
          break;
        }

    gc_->init(init_q);
    gc_->running = true;

    for (size_t i = 0; i < ref_order.size(); ++i)
    {
      auto idx = gc_->robot().jointIndexByName(ref_order[i]);
      mc_rtc::log::info("[KortexBridge] Joint {} | init_q: {} | mbc.q: {}",
                        ref_order[i], init_q[i], gc_->robot().mbc().q[idx][0]);
    }

    initialized_ = true;
    timer_ = this->create_wall_timer(10ms, std::bind(&KortexMcRtcBridge::controlLoop, this));
    mc_rtc::log::success("[KortexBridge] mc_rtc seeded. Control loop started!");
  }

  void controlLoop()
  {
    if (!initialized_) return;

    // Check if wrench has gone stale (>100ms without update = no force)
    bool has_fresh_wrench = false;
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      if (latest_wrench_)
      {
        auto age = (this->now() - last_wrench_time_).seconds();
        has_fresh_wrench = (age < 0.1);

        if (has_fresh_wrench)
        {
          std::map<std::string, sva::ForceVecd> wrenches;
          auto & w = latest_wrench_->wrench;
          wrenches["EEForceSensor"] = sva::ForceVecd(
              Eigen::Vector3d(w.torque.x, w.torque.y, w.torque.z),
              Eigen::Vector3d(w.force.x,  w.force.y,  w.force.z));
          gc_->setWrenches(wrenches);

          static int log_count = 0;
          if(++log_count % 500 == 0)
            mc_rtc::log::info("[KortexBridge] Wrench fed: force=({},{},{}) torque=({},{},{})",
              w.force.x, w.force.y, w.force.z,
              w.torque.x, w.torque.y, w.torque.z);
        }
        else
        {
          // Stale — zero out the wrench
          std::map<std::string, sva::ForceVecd> wrenches;
          wrenches["EEForceSensor"] = sva::ForceVecd(
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
          gc_->setWrenches(wrenches);

          if (wrench_active_)
          {
            mc_rtc::log::info("[KortexBridge] Wrench stopped — sending hold.");
            wrench_active_ = false;
          }
        }
      }
    }

    if (has_fresh_wrench && !wrench_active_)
    {
      mc_rtc::log::info("[KortexBridge] Wrench active.");
      wrench_active_ = true;
    }

    if (gc_->run())
    {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      for (size_t i = 0; i < traj.joint_names.size(); ++i)
      {
        if (gc_->robot().hasJoint(traj.joint_names[i]))
        {
          auto idx = gc_->robot().jointIndexByName(traj.joint_names[i]);
          pt.positions.push_back(gc_->robot().mbc().q[idx][0]);
        }
        else
          pt.positions.push_back(0.0);
        pt.velocities.push_back(0.0);
      }
      pt.time_from_start.nanosec = 20'000'000; // 20ms
      traj.points.push_back(pt);
      pub_->publish(traj);
    }
  }

  std::shared_ptr<mc_control::MCGlobalController> gc_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KortexMcRtcBridge>());
  rclcpp::shutdown();
  return 0;
}

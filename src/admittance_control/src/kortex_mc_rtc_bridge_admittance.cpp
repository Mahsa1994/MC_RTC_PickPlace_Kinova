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
        "/joint_trajectory_controller/joint_trajectory", 10);

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
        });

    mc_rtc::log::info("[KortexBridge] Waiting for first /joint_states...");
    mc_rtc::log::info("[KortexBridge] Subscribed to /EEForceSensor");
  }

private:
  std::vector<double> last_published_q_ = std::vector<double>(6, 0.0);
  int settle_ticks_ = 0;
  static constexpr int SETTLE_TICKS_MAX = 50;
  std::mutex init_mutex_;
  std::atomic<bool> initialized_{false};
  std::vector<double> last_alpha_ = std::vector<double>(6, 0.0);
  double dt_ = 0.01;

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  geometry_msgs::msg::WrenchStamped::SharedPtr latest_wrench_;
  std::mutex wrench_mutex_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (initialized_)
    {
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> enc_q(ref_order.size(), 0.0);
      std::vector<double> enc_alpha(ref_order.size(), 0.0);
      for (size_t i = 0; i < ref_order.size(); ++i)
      {
        for (size_t j = 0; j < msg->name.size(); ++j)
        {
          if (msg->name[j] == ref_order[i])
          {
            enc_q[i]    = msg->position[j];
            enc_alpha[i] = msg->velocity[j];
            break;
          }
        }
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
    {
      for (size_t j = 0; j < msg->name.size(); ++j)
      {
        if (msg->name[j] == ref_order[i])
        {
          init_q[i] = msg->position[j];
          break;
        }
      }
    }

    gc_->init(init_q);
    last_published_q_ = init_q;
    gc_->running = true;
    dt_ = gc_->timestep();

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

    // Feed force sensor into mc_rtc
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      if (latest_wrench_)
      {
        std::map<std::string, sva::ForceVecd> wrenches;
        auto & w = latest_wrench_->wrench;
        wrenches["EEForceSensor"] = sva::ForceVecd(
            Eigen::Vector3d(w.torque.x, w.torque.y, w.torque.z),
            Eigen::Vector3d(w.force.x,  w.force.y,  w.force.z));
        gc_->setWrenches(wrenches);
      }
    }

    if (gc_->run())
    {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      trajectory_msgs::msg::JointTrajectoryPoint pt1;
      bool intent_to_move = false;

      for (size_t i = 0; i < traj.joint_names.size(); ++i)
      {
        if (gc_->robot().hasJoint(traj.joint_names[i]))
        {
          auto mbc_idx = gc_->robot().jointIndexByName(traj.joint_names[i]);
          double q     = gc_->robot().mbc().q[mbc_idx][0];
          double alpha = gc_->robot().mbc().alpha[mbc_idx][0];
          pt1.positions.push_back(q);
          pt1.velocities.push_back(alpha);
          last_alpha_[i] = alpha;
          if (std::abs(q - last_published_q_[i]) > 1e-5 || std::abs(alpha) > 1e-4)
            intent_to_move = true;
        }
        else
        {
          pt1.positions.push_back(0.0);
          pt1.velocities.push_back(0.0);
          last_alpha_[i] = 0.0;
        }
      }

      if (intent_to_move) settle_ticks_ = 0;
      else                settle_ticks_++;

      if (settle_ticks_ == SETTLE_TICKS_MAX)
      {
        trajectory_msgs::msg::JointTrajectory hold;
        hold.joint_names = traj.joint_names;
        trajectory_msgs::msg::JointTrajectoryPoint hold_pt;
        hold_pt.positions  = last_published_q_;
        hold_pt.velocities = std::vector<double>(last_published_q_.size(), 0.0);
        hold_pt.time_from_start.nanosec = 200'000'000;
        hold.points.push_back(hold_pt);
        pub_->publish(hold);
      }

      if (settle_ticks_ < SETTLE_TICKS_MAX)
      {
        pt1.time_from_start.nanosec = 20'000'000;
        traj.points.push_back(pt1);

        trajectory_msgs::msg::JointTrajectoryPoint pt2;
        for (size_t i = 0; i < pt1.positions.size(); ++i)
        {
          pt2.positions.push_back(pt1.positions[i] + last_alpha_[i] * dt_);
          pt2.velocities.push_back(last_alpha_[i]);
        }
        pt2.time_from_start.nanosec = 40'000'000;
        traj.points.push_back(pt2);

        double max_alpha = 0.0, max_delta = 0.0;
        for (size_t i = 0; i < last_alpha_.size(); ++i)
          max_alpha = std::max(max_alpha, std::abs(last_alpha_[i]));
        for (size_t i = 0; i < pt1.positions.size(); ++i)
          max_delta = std::max(max_delta, std::abs(pt1.positions[i] - last_published_q_[i]));
        const bool genuinely_stopping = (max_alpha < 0.002 && max_delta < 0.0005);

        trajectory_msgs::msg::JointTrajectoryPoint pt3;
        for (size_t i = 0; i < pt2.positions.size(); ++i)
        {
          double step = (max_delta > 0.0005 && !genuinely_stopping) ? last_alpha_[i] * dt_ : 0.0;
          pt3.positions.push_back(pt2.positions[i] + step);
          pt3.velocities.push_back(0.0);
        }
        pt3.time_from_start.nanosec = 60'000'000;
        traj.points.push_back(pt3);

        pub_->publish(traj);
        last_published_q_ = pt1.positions;
      }
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

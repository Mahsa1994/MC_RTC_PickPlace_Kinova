#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>

#include <atomic>

using namespace std::chrono_literals;

class KortexMcRtcBridge : public rclcpp::Node
{
public:
  KortexMcRtcBridge() : Node("kortex_mc_rtc_bridge")
  {
    // Publish directly to Kinova's Trajectory Controller
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    gc_ = std::make_shared<mc_control::MCGlobalController>();

    // Listen to Kinova's true hardware state
//    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//        "/joint_states", 10,
//        std::bind(&KortexMcRtcBridge::jointStateCallback, this, std::placeholders::_1));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)
        .durability(rclcpp::DurabilityPolicy::Volatile);
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos,
        std::bind(&KortexMcRtcBridge::jointStateCallback, this, std::placeholders::_1));

    mc_rtc::log::info("[KortexBridge] Waiting for first /joint_states from real Kinova...");
  }

private:

  std::vector<double> last_published_q_ = std::vector<double>(6, 0.0);
  int idle_ticks_ = 0;

  // The control loop checks this before running to avoid a race condition.
  std::atomic<bool> initialized_{false};


  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if(!initialized_)
    {
    RCLCPP_INFO(this->get_logger(), "Initializing mc_rtc...");

      // 2. Map real joint states to mc_rtc's internal order
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> init_q(ref_order.size(), 0.0);

      for(size_t i = 0; i < ref_order.size(); ++i) {
        for(size_t j = 0; j < msg->name.size(); ++j) {
          if(msg->name[j] == ref_order[i]) {
            init_q[i] = msg->position[j];
            break;
          }
        }
      }

      // 3. Seed FSM with true hardware position (FIXES THE JUMP!)
      // to the current robot configuration, preventing a jump on startup.
      // Explicitly calling init() is sufficient — it triggers MCController::reset() internally.
      // If you ever switch controllers at runtime, you must call reset() again manually.
      gc_->init(init_q);
      gc_->running = true;

      // add this log line right after gc_->run() in controlLoop(). If the robot is stationary
      // but mbc().q drifts away from enc_q below, your observer pipeline is NOT closing the loop.
      // Log both here to compare at startup:
      auto ref = gc_->robot().refJointOrder();
      for(size_t i = 0; i < ref.size(); ++i) {
        auto idx = gc_->robot().jointIndexByName(ref[i]);
        mc_rtc::log::info("[KortexBridge] Joint {} | init_q (encoder): {} | mbc.q (QP seed): {}",
          ref[i], init_q[i], gc_->robot().mbc().q[idx][0]);
	}
      // 4. Start Control Loop at 200 Hz (dt = 0.005s) - changed from 5 to 10
      //timer_ = this->create_wall_timer(10ms, std::bind(&KortexMcRtcBridge::controlLoop, this));
      //mc_rtc::log::success("[KortexBridge] mc_rtc seeded with real robot state. 200Hz Control loop started!");
    }
    else
    {
      // Keep encoders updated for FSM closed-loop observation
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> enc_q(ref_order.size(), 0.0);
      std::vector<double> enc_alpha(ref_order.size(), 0.0);
      for(size_t i = 0; i < ref_order.size(); ++i) {
        for(size_t j = 0; j < msg->name.size(); ++j) {
          if(msg->name[j] == ref_order[i]) { 
             enc_q[i] = msg->position[j];
             enc_alpha[i] = msg->velocity[j];
             break; }
        }
      }
      gc_->setEncoderValues(gc_->robot().name(), enc_q);
      gc_->setEncoderVelocities(gc_->robot().name(), enc_alpha);
    }
  }


void controlLoop()
  {

    if(!initialized_) return;

    if(gc_->run())
    {

      // FIX 2 (HOW TO CHECK): Uncomment these lines to compare QP output vs real encoder state.
      // If mbc().q and enc_q diverge while the robot is still, your observer is open-loop.
      // auto ref = gc_->robot().refJointOrder();
      // for(size_t i = 0; i < ref.size(); ++i) {
      //   auto idx = gc_->robot().jointIndexByName(ref[i]);
      //   mc_rtc::log::info("[QP vs ENC] {} | mbc.q: {} | encoder: {}",
      //     ref[i], gc_->robot().mbc().q[idx][0], gc_->robot().encoderValues()[i]);
      // }
      
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      trajectory_msgs::msg::JointTrajectoryPoint pt1;
      
      // Keep track of where we last told the robot to go
      //static std::vector<double> last_published_q(6, 0.0);
      bool intent_to_move = false;

      for(size_t i = 0; i < traj.joint_names.size(); ++i)
      {
        if(gc_->robot().hasJoint(traj.joint_names[i])) {
          auto mbc_idx = gc_->robot().jointIndexByName(traj.joint_names[i]);
          double q = gc_->robot().mbc().q[mbc_idx][0];
          double alpha = gc_->robot().mbc().alpha[mbc_idx][0];
          
          pt1.positions.push_back(q);
          pt1.velocities.push_back(alpha);
          
          // If the position changed by even a micro-radian, or velocity is active, we are moving!
        if(std::abs(q - last_published_q_[i]) > 1e-5 || std::abs(alpha) > 1e-4) {
            intent_to_move = true;
          }
          //if(std::abs(q - last_published_q[i]) > 1e-5 || std::abs(alpha) > 1e-4) {
          //  intent_to_move = true;
          //}
        } else {
          pt1.positions.push_back(0.0);
          pt1.velocities.push_back(0.0);
        }
      }

//      static int idle_ticks = 0;
      if(intent_to_move) {
        idle_ticks_ = 0; // Reset counter immediately when FSM starts moving
      } else {
        idle_ticks_++;
      }

      // Publish during movement, AND for 50 ticks (250ms) after stopping to let the arm settle smoothly
      if(idle_ticks_ < 50)
      {
        pt1.time_from_start.sec = 0;
        pt1.time_from_start.nanosec = 10000000; //5000000;
        traj.points.push_back(pt1);

        // Dummy stop point to bypass ROS 2 strict velocity checks
        trajectory_msgs::msg::JointTrajectoryPoint pt2;
        pt2.positions = pt1.positions;
        pt2.velocities = std::vector<double>(pt1.positions.size(), 0.0);
        pt2.time_from_start.sec = 0;
        pt2.time_from_start.nanosec = 20000000; //100000000; // 100ms in the future
        traj.points.push_back(pt2);

        pub_->publish(traj);
        last_published_q_ = pt1.positions; // Save for next loop
      }
    }
  }



  std::shared_ptr<mc_control::MCGlobalController> gc_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KortexMcRtcBridge>());
  rclcpp::shutdown();
  return 0;
}

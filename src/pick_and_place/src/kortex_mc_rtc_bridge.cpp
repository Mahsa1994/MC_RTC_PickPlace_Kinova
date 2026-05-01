#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>

using namespace std::chrono_literals;

class KortexMcRtcBridge : public rclcpp::Node
{
public:
  KortexMcRtcBridge() : Node("kortex_mc_rtc_bridge")
  {
    // Publish directly to Kinova's Trajectory Controller
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    
    // Listen to Kinova's true hardware state
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&KortexMcRtcBridge::jointStateCallback, this, std::placeholders::_1));

    mc_rtc::log::info("[KortexBridge] Waiting for first /joint_states from real Kinova...");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if(!gc_)
    {
      // 1. Initialize mc_rtc
      // mc_rtc::Configuration config; // Loads your ~/.config/mc_rtc/mc_rtc.yaml FSM
//      gc_ = std::make_shared<mc_control::MCGlobalController>(config);
      gc_ = std::make_shared<mc_control::MCGlobalController>();

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
      gc_->init(init_q);
      gc_->running = true;

      // 4. Start Control Loop at 200 Hz (dt = 0.005s)
      timer_ = this->create_wall_timer(5ms, std::bind(&KortexMcRtcBridge::controlLoop, this));
      mc_rtc::log::success("[KortexBridge] mc_rtc seeded with real robot state. 200Hz Control loop started!");
    }
    else
    {
      // Keep encoders updated for FSM closed-loop observation
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> enc_q(ref_order.size(), 0.0);
      for(size_t i = 0; i < ref_order.size(); ++i) {
        for(size_t j = 0; j < msg->name.size(); ++j) {
          if(msg->name[j] == ref_order[i]) { enc_q[i] = msg->position[j]; break; }
        }
      }
      gc_->setEncoderValues(gc_->robot().name(), enc_q);
    }
  }

  void controlLoop()
  {
    if(gc_->run())
    {
      // Send 200Hz high-density points to Kinova driver (FIXES THE SHAKE!)
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      for(const auto & jn : traj.joint_names)
      {
          if(gc_->robot().hasJoint(jn)) {
             auto mbc_idx = gc_->robot().jointIndexByName(jn);
             pt.positions.push_back(gc_->robot().mbc().q[mbc_idx][0]);
             pt.velocities.push_back(gc_->robot().mbc().alpha[mbc_idx][0]);
   //          // 1. Get the joint's numeric index from its string name
     //        auto joint_idx = gc_->robot().jointIndexByName(jn);
       //      // 2. Get the index of that joint in the MBC state vector
       //      auto mbc_idx = gc_->robot().jointIndexInMBC(joint_idx); 
        //     pt.positions.push_back(gc_->robot().mbc().q[mbc_idx][0]);
         //    pt.velocities.push_back(gc_->robot().mbc().alpha[mbc_idx][0]);
        } else {
          pt.positions.push_back(0.0);
          pt.velocities.push_back(0.0);
        }
      }

      // Tell driver this point happens in exactly 5ms
      pt.time_from_start.sec = 0;
      pt.time_from_start.nanosec = 5000000; 

      traj.points.push_back(pt);
      pub_->publish(traj);
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

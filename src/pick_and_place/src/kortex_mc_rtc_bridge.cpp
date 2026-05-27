#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
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
    // pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    //    "/joint_group_position_controller/commands", 1);

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
  int settle_ticks_ = 0;
  static constexpr int SETTLE_TICKS_MAX = 50;

  std::mutex init_mutex_;

  // The control loop checks this before running to avoid a race condition.
  std::atomic<bool> initialized_{false};

  std::vector<double> last_alpha_ = std::vector<double>(6, 0.0);
  double dt_ = 0.01;

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
            enc_q[i] = msg->position[j];
            enc_alpha[i] = msg->velocity[j];
            break;
          }
        }
      }
      gc_->setEncoderValues(gc_->robot().name(), enc_q);
      gc_->setEncoderVelocities(gc_->robot().name(), enc_alpha);
      return; // EXIT HERE — never fall through to init block
    }

    // SLOW PATH — first-time init only.
    // Lock so that even if multiple callbacks queued up at 1000Hz
    // and all passed the `if(initialized_)` check above simultaneously,
    // only ONE of them executes the init block.
    std::lock_guard<std::mutex> lock(init_mutex_);

    // DOUBLE-CHECK after acquiring the lock.
    // The 2nd, 3rd, 4th... callbacks that were queued will reach here
    // after the first one finishes and set initialized_ = true.
    // Without this second check, they would ALL run init — which is
    // exactly the bug you saw (5x init in the log).
    if (initialized_)
      return;

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
    gc_->running = true;
    dt_ = gc_->timestep();

    // Log encoder vs QP seed to confirm correct seeding
    for (size_t i = 0; i < ref_order.size(); ++i)
    {
      auto idx = gc_->robot().jointIndexByName(ref_order[i]);
      mc_rtc::log::info("[KortexBridge] Joint {} | init_q (encoder): {} | mbc.q (QP seed): {}",
                        ref_order[i], init_q[i], gc_->robot().mbc().q[idx][0]);
    }

    // SET THE FLAG LAST — only after everything above is complete.
    // Any callback that arrives now and checks initialized_ will
    // go straight to the fast path above.
    initialized_ = true;

    // Create the control loop timer only once, after full init.
    timer_ = this->create_wall_timer(10ms, std::bind(&KortexMcRtcBridge::controlLoop, this));
    mc_rtc::log::success("[KortexBridge] mc_rtc seeded. Control loop started!");
  }

  void controlLoop()
  {

    if (!initialized_)
      return;

    if (gc_->run())
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
      // static std::vector<double> last_published_q(6, 0.0);
      bool intent_to_move = false;

      for (size_t i = 0; i < traj.joint_names.size(); ++i)
      {
        if (gc_->robot().hasJoint(traj.joint_names[i]))
        {
          auto mbc_idx = gc_->robot().jointIndexByName(traj.joint_names[i]);
          double q = gc_->robot().mbc().q[mbc_idx][0];
          double alpha = gc_->robot().mbc().alpha[mbc_idx][0];

          pt1.positions.push_back(q);
          pt1.velocities.push_back(alpha);

          last_alpha_[i] = alpha;
          // If the position changed by even a micro-radian, or velocity is active, we are moving!
          if (std::abs(q - last_published_q_[i]) > 1e-5 || std::abs(alpha) > 1e-4)
          {
            intent_to_move = true;
          }
          // if(std::abs(q - last_published_q[i]) > 1e-5 || std::abs(alpha) > 1e-4) {
          //   intent_to_move = true;
          // }
        }
        else
        {
          pt1.positions.push_back(0.0);
          pt1.velocities.push_back(0.0);
          last_alpha_[i] = 0.0;
        }
      }

      //      static int idle_ticks = 0;
      /*if (intent_to_move)
      {
        settle_ticks_ = 0; // Reset counter immediately when FSM starts moving
      }
      else
      {
        settle_ticks_++;
      }*/
      if (intent_to_move)
      {
        settle_ticks_ = 0;
      }
      else
      {
        settle_ticks_++;
      }

      // When the bridge stops publishing (settle window expired),
      // send one explicit hold message so the JTC has a clean zero-velocity
      // target at the exact final position. Without this, the last buffered
      // pt2/pt3 forward projections execute and cause the arm to drift past
      // the target after mc_rtc has already converged.
      if(settle_ticks_ == SETTLE_TICKS_MAX)
      {
        trajectory_msgs::msg::JointTrajectory hold;
        hold.joint_names = traj.joint_names;
        trajectory_msgs::msg::JointTrajectoryPoint hold_pt;
        hold_pt.positions = last_published_q_;
        hold_pt.velocities = std::vector<double>(last_published_q_.size(), 0.0);
        hold_pt.time_from_start.sec = 0;
        hold_pt.time_from_start.nanosec = 200'000'000; // 200ms — long window, zero velocity
        hold.points.push_back(hold_pt);
        pub_->publish(hold);
      }

      if (settle_ticks_ < SETTLE_TICKS_MAX)
      {
        // pt1: current mc_rtc output — 20ms
        pt1.time_from_start.sec = 0;
        pt1.time_from_start.nanosec = 20'000'000;
        traj.points.push_back(pt1);

        // pt2: one dt forward projection with velocity — 40ms
        trajectory_msgs::msg::JointTrajectoryPoint pt2;
        for (size_t i = 0; i < pt1.positions.size(); ++i)
        {
          pt2.positions.push_back(pt1.positions[i] + last_alpha_[i] * dt_);
          pt2.velocities.push_back(last_alpha_[i]);
        }
        pt2.time_from_start.sec = 0;
        pt2.time_from_start.nanosec = 40'000'000;
        traj.points.push_back(pt2);

        // pt3: terminal zero-velocity point — 60ms
        // Required by JTC: last point in trajectory must have zero velocity.
        // pt3 position = pt2 position (hold in place), velocity = 0.
        trajectory_msgs::msg::JointTrajectoryPoint pt3;
        
        /* double max_alpha = 0.0;
        for(size_t i = 0; i < last_alpha_.size(); ++i)
          max_alpha = std::max(max_alpha, std::abs(last_alpha_[i]));
        const bool genuinely_stopping = (max_alpha < 0.002); // rad/s */

        double max_alpha = 0.0;
        for(size_t i = 0; i < last_alpha_.size(); ++i)
          max_alpha = std::max(max_alpha, std::abs(last_alpha_[i]));

        // Check how much pt1 moved from last published position.
        double max_delta = 0.0;
        for(size_t i = 0; i < pt1.positions.size(); ++i)
          max_delta = std::max(max_delta,
                               std::abs(pt1.positions[i] - last_published_q_[i]));

        // Genuinely stopping: velocity is tiny AND position is barely changing.
        const bool genuinely_stopping = (max_alpha < 0.002 && max_delta < 0.0005);


        for(size_t i = 0; i < pt2.positions.size(); ++i)
        {
          if(genuinely_stopping)
          {
            // Motion ending — zero velocity is safe, JTC will hold cleanly
            pt3.positions.push_back(pt2.positions[i]);
            pt3.velocities.push_back(0.0);
          }
          else
          {
            //// Still moving — project position forward one more step so the
            //pt3.positions.push_back(pt2.positions[i] + last_alpha_[i] * dt_);
            //pt3.velocities.push_back(0.0);
            // Only project forward if the step is meaningful.
            double step = last_alpha_[i] * dt_;
            if(max_delta > 0.0005)
              pt3.positions.push_back(pt2.positions[i] + step);
            else
              pt3.positions.push_back(pt2.positions[i]); // hold — arm nearly stopped
            pt3.velocities.push_back(0.0);
          }
        }
        pt3.time_from_start.sec = 0;
        pt3.time_from_start.nanosec = 60'000'000;
        traj.points.push_back(pt3);

        pub_->publish(traj);
        last_published_q_ = pt1.positions;
      }
      // Publish during movement, AND for 50 ticks (250ms) after stopping to let the arm settle smoothly
      // if (settle_ticks_ < SETTLE_TICKS_MAX)
      // {
      //   pt1.time_from_start.sec = 0;
      //   pt1.time_from_start.nanosec = 20000000; // 5000000;
      //   traj.points.push_back(pt1);

      //   // Dummy stop point to bypass ROS 2 strict velocity checks
      //   trajectory_msgs::msg::JointTrajectoryPoint pt2;
      //   const double dt = 0.01;
      //   for (size_t i = 0; i < pt1.positions.size(); ++i)
      //   {
      //     // Forward-project position by one dt step
      //     pt2.positions.push_back(pt1.positions[i] + last_alpha_[i] * dt);
      //     // Carry the same velocity forward — let the JTC interpolate
      //     // the deceleration naturally when mc_rtc reduces alpha
      //     pt2.velocities.push_back(last_alpha_[i]);
      //   }

      //   // [FIX 7] pt2 at 40ms (2× pt1, 4× dt).
      //   // Gives the JTC a 40ms lookahead window, wide enough for the
      //   // spline interpolator to produce smooth velocity profiles between
      //   // updates instead of bang-bang position steps.
      //   pt2.time_from_start.sec = 0;
      //   pt2.time_from_start.nanosec = 40'000'000; // 40ms

      //   traj.points.push_back(pt2);

      //   pub_->publish(traj);
      //   last_published_q_ = pt1.positions;
      // }
    }
  }

  std::shared_ptr<mc_control::MCGlobalController> gc_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  //  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <RBDyn/ID.h>
#include <RBDyn/Jacobian.h>
#include <Eigen/QR>

#include <mutex>
#include <atomic>
#include <vector>
#include <map>
#include <thread>

using namespace std::chrono_literals;

class KortexMcRtcBridge : public rclcpp::Node
{
public:
  KortexMcRtcBridge() : Node("kortex_mc_rtc_bridge_impedance")
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

    auto ref_order = gc_->robot().refJointOrder();
    latest_efforts_.resize(ref_order.size(), 0.0);

    mc_rtc::log::info("[KortexBridge] Waiting for first /joint_states...");
    mc_rtc::log::info("[KortexBridge] Operating in joint torque estimation mode.");
  }

  /// Destructor
  ~KortexMcRtcBridge()
  {
    mc_rtc::log::info("[KortexBridge] Shutting down. Sending final hold command...");
    if (initialized_ && pub_)
    {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      
      // Query the real robot's actual joint encoders
      const auto & real_robot = gc_->realRobot();
      for (const auto & name : traj.joint_names)
      {
        if (real_robot.hasJoint(name))
        {
          auto idx = real_robot.jointIndexByName(name);
          pt.positions.push_back(real_robot.mbc().q[idx][0]);
        }
        else
        {
          pt.positions.push_back(0.0);
        }
        pt.velocities.push_back(0.0);
      }
      pt.time_from_start.nanosec = 50'000'000; // 50 ms hold buffer
      traj.points.push_back(pt);
      pub_->publish(traj);
      
      // Give the ROS 2 publisher a tiny moment to send the message before the socket closes
      std::this_thread::sleep_for(100ms);
    }
  }

private:
  std::mutex init_mutex_;
  std::atomic<bool> initialized_{false};

  std::vector<double> latest_efforts_;
  std::mutex effort_mutex_;

  // ── Software-level Wrench Tare Variables
  bool tared_{false};
  int tare_ticks_{0};
  Eigen::Vector3d bias_force_world_{0.0, 0.0, 0.0};
  Eigen::Vector3d bias_moment_world_{0.0, 0.0, 0.0};

  // ── Helper: build a map from joint name → DOF start index in the nrDof vector
  // RBDyn does not expose jointVelocityIndex(); we reconstruct it by walking
  // the joint list and accumulating DOF counts.
  std::map<std::string, int> buildJointDofMap(const rbd::MultiBody & mb)
  {
    std::map<std::string, int> dof_map;
    int dof_offset = 0;
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      const auto & j = mb.joint(i);
      // Only record joints with DOFs (skip fixed/root anchor joints)
      if (j.dof() > 0)
        dof_map[j.name()] = dof_offset;
      dof_offset += j.dof();
    }
    return dof_map;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (initialized_)
    {
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> enc_q(ref_order.size(), 0.0);
      std::vector<double> enc_alpha(ref_order.size(), 0.0);
      std::vector<double> enc_tau(ref_order.size(), 0.0);

      for (size_t i = 0; i < ref_order.size(); ++i)
        for (size_t j = 0; j < msg->name.size(); ++j)
          if (msg->name[j] == ref_order[i])
          {
            enc_q[i]     = msg->position[j];
            enc_alpha[i] = msg->velocity[j];
            enc_tau[i]   = (j < msg->effort.size()) ? msg->effort[j] : 0.0;
            break;
          }

      gc_->setEncoderValues(gc_->robot().name(), enc_q);
      gc_->setEncoderVelocities(gc_->robot().name(), enc_alpha);
      gc_->setJointTorques(gc_->robot().name(), enc_tau);

      {
        std::lock_guard<std::mutex> lock(effort_mutex_);
        latest_efforts_ = enc_tau;
      }
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

    for (const auto & fs : gc_->robot().forceSensors())
    {
        mc_rtc::log::info("[KortexBridge] Registered Robot Force Sensor: '{}' on body: '{}'",
                    fs.name(), fs.parentBody());
    }

    gc_->running = true;

    for (size_t i = 0; i < ref_order.size(); ++i)
    {
      auto idx = gc_->robot().jointIndexByName(ref_order[i]);
      mc_rtc::log::info("[KortexBridge] Joint {} | init_q: {} | mbc.q: {}",
                        ref_order[i], init_q[i], gc_->robot().mbc().q[idx][0]);
    }

    initialized_ = true;
    timer_ = this->create_wall_timer(1ms, std::bind(&KortexMcRtcBridge::controlLoop, this)); //10ms
    mc_rtc::log::success("[KortexBridge] mc_rtc seeded. Control loop started!");
  }

  void controlLoop()
  {
    if (!initialized_) return;

    const auto & robot = gc_->robot();
    const auto & mb    = robot.mb();

    // Build joint-name → DOF-index map
    auto dof_map = buildJointDofMap(mb);

    // �1. Map measured joint torques -  full nrDof vector (size 12)
    Eigen::VectorXd tau_meas = Eigen::VectorXd::Zero(mb.nrDof());
    {
      std::lock_guard<std::mutex> lock(effort_mutex_);
      auto ref_order = robot.refJointOrder();
      for (size_t i = 0; i < ref_order.size(); ++i)
      {
        auto it = dof_map.find(ref_order[i]);
        if (it != dof_map.end() && i < latest_efforts_.size())
          tau_meas[it->second] = latest_efforts_[i];
      }
    }

    // 2. Inverse dynamics to get gravity + Coriolis bias torques
    rbd::MultiBodyConfig mbc_id = robot.mbc();
    mbc_id.gravity = Eigen::Vector3d(0, 0, 9.81); 
    for (auto & ad : mbc_id.alphaD)
      std::fill(ad.begin(), ad.end(), 0.0);        

    rbd::InverseDynamics id(mb);
    id.inverseDynamics(mb, mbc_id);

    // 3. Extract bias torques into nrDof vector (size 12)
    Eigen::VectorXd tau_bias = Eigen::VectorXd::Zero(mb.nrDof());
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      auto it = dof_map.find(mb.joint(i).name());
      if (it != dof_map.end() && !mbc_id.jointTorque[i].empty())
        tau_bias[it->second] = mbc_id.jointTorque[i][0];
    }

    // 4. Full Jacobian of end-effector frame (size 6 × 12)
    rbd::Jacobian jac(mb, "tool_frame"); 
    Eigen::MatrixXd J_full = jac.jacobian(mb, robot.mbc()); 

    // �5. Extract only active joint components (size 6 × 6)
    auto ref_order = robot.refJointOrder();
    Eigen::MatrixXd J_active = Eigen::MatrixXd::Zero(6, ref_order.size());
    Eigen::VectorXd tau_ext_active = Eigen::VectorXd::Zero(ref_order.size());

    for (size_t i = 0; i < ref_order.size(); ++i)
    {
      auto it = dof_map.find(ref_order[i]);
      if (it != dof_map.end())
      {
        int col_idx = it->second;
        J_active.col(i) = J_full.col(col_idx);
//        tau_ext_active[i] = tau_meas[col_idx] - tau_bias[col_idx];
        tau_ext_active[i] = tau_bias[col_idx] - tau_meas[col_idx];

      }
    }

    // �6. Solve J_active�- F_world = tau_ext_active (size 6 × 6 solve)
    Eigen::VectorXd F_world = J_active.transpose()
                                .completeOrthogonalDecomposition()
                                .solve(tau_ext_active);

    Eigen::Vector3d moment_world(F_world[0], F_world[1], F_world[2]);
    Eigen::Vector3d force_world(F_world[3], F_world[4], F_world[5]);

    static int startup_delay_ticks = 0;
    if (startup_delay_ticks < 3000)
    {
      // During the 3-second startup delay, we keep command tracking active but force wrench to 0
      startup_delay_ticks++;
      force_world.setZero();
      moment_world.setZero();
    }
    else
    {
      // Dynamic World-Frame Tare (Zeroing) - runs AFTER the 3-second delay is complete
      if (!tared_)
      {
        if (tare_ticks_ < 200) // Collect 200 ticks of quiet baseline data
        {
          bias_force_world_  += force_world;
          bias_moment_world_ += moment_world;
          tare_ticks_++;
        }
        else
        {
          bias_force_world_  /= tare_ticks_;
          bias_moment_world_ /= tare_ticks_;
          tared_ = true;
          mc_rtc::log::success("[KortexBridge] World-frame wrench tared successfully!");
        }
        force_world.setZero();
        moment_world.setZero();
      }
      else
      {
        // Subtract tared gravity offsets
        force_world  -= bias_force_world_;
        moment_world -= bias_moment_world_;
      }
    }

    // �7. Rotate the tared wrench from World Frame to Sensor Local Frame
    Eigen::Matrix3d R_world_sensor = robot.bodyPosW("tool_frame").rotation();
    
    Eigen::Vector3d moment_sensor = R_world_sensor * moment_world;
    Eigen::Vector3d force_sensor  = R_world_sensor * force_world;

     static Eigen::Vector3d filtered_force = Eigen::Vector3d::Zero();
    static Eigen::Vector3d filtered_moment = Eigen::Vector3d::Zero();
    if (tared_)
    {
      // 1. Accumulate the filter state normally (no resetting here!)
      filtered_force = 0.90 * filtered_force + 0.10 * force_sensor;
      filtered_moment = 0.90 * filtered_moment + 0.10 * moment_sensor;

      // 2. Create temporary copies for output thresholding
      Eigen::Vector3d output_force = filtered_force;
      Eigen::Vector3d output_moment = filtered_moment;

      // 3. Apply Deadband on the temporary copies
      if (output_force.norm() < 1.5) {
        output_force.setZero();
      }
      if (output_moment.norm() < 0.5) {
        output_moment.setZero();
      }

      // 4. Update the active sensor readings to send to mc_rtc
      force_sensor = output_force;
      moment_sensor = output_moment;
    }

    // ── 8. Inject estimated wrench into mc_rtc
    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches["EEForceSensor"] = sva::ForceVecd(moment_sensor, force_sensor);

   // �Velocity-scaled wrench gate
   // When the arm is moving fast, the ID-based estimator is unreliable.
   // Scale the injected wrench toward zero as joint velocity increases.
     double max_qd = 0.0;
for (int i = 0; i < mb.nrJoints(); ++i)
{
  if (mb.joint(i).dof() == 1)  // only 1-DOF revolute joints
  {
    double qd = std::abs(robot.mbc().alpha[i][0]);
    if (qd > max_qd) max_qd = qd;
  }
}
// Gate: full wrench below 0.05 rad/s, zero above 0.15 rad/s
const double qd_low  = 0.05;
const double qd_high = 0.15;
double gate = 1.0 - std::clamp((max_qd - qd_low) / (qd_high - qd_low), 0.0, 1.0);
force_sensor  *= gate;
moment_sensor *= gate;


//    wrenches["EEForceSensor"] = sva::ForceVecd(filtered_moment, filtered_force);
    gc_->setWrenches(wrenches);

    static int log_count = 0;
    if (++log_count % 500 == 0)
    {
      mc_rtc::log::info(
          "[KortexBridge] Est. wrench (Local) — force: ({:.2f}, {:.2f}, {:.2f}) N  "
          "moment: ({:.2f}, {:.2f}, {:.2f}) Nm",
          force_sensor.x(), force_sensor.y(), force_sensor.z(),
          moment_sensor.x(), moment_sensor.y(), moment_sensor.z());
    }

    // ── 9. Run controller and publish joint trajectory
    if (gc_->run())
    {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      for (const auto & name : traj.joint_names)
      {
        if (gc_->robot().hasJoint(name))
        {
          auto idx = gc_->robot().jointIndexByName(name);
          pt.positions.push_back(gc_->robot().mbc().q[idx][0]);
        }
        else
          pt.positions.push_back(0.0);
        pt.velocities.push_back(0.0);
      }
      pt.time_from_start.nanosec = 20'000'000; // 20 ms
      traj.points.push_back(pt);
      pub_->publish(traj); // Commented out for initial dry-run safety
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

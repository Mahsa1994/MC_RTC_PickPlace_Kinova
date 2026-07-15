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

    dry_run_    = this->declare_parameter("dry_run", false);
    delta_max_  = this->declare_parameter("delta_max", 0.05); 
    pub_decim_  = this->declare_parameter("publish_decimation", 10); // 1kHz/10 = 100Hz
    loop_dt_ = this->declare_parameter("loop_dt", 0.001);   // seconds; sim keeps 0.001

    torque_sign_    = this->declare_parameter("torque_sign", 1.0);   // flip to -1.0 if inverted on real
    deadband_force_ = this->declare_parameter("deadband_force", 1.5);  // sim value; real: start 6.0
    deadband_moment_= this->declare_parameter("deadband_moment", 0.5); // sim value; real: start 1.5

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
      const auto &real_robot = gc_->realRobot();
      for (const auto &name : traj.joint_names)
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

      // Give the ROS publisher a moment to send the message before the socket closes
      std::this_thread::sleep_for(100ms);
    }
  }

private:
  std::mutex init_mutex_;
  std::atomic<bool> initialized_{false};

  std::vector<double> latest_efforts_;
  std::mutex effort_mutex_;

  // Software-level Wrench Tare Variables
  bool tared_{false};
  int tare_ticks_{0};
  Eigen::Vector3d bias_force_world_{0.0, 0.0, 0.0};
  Eigen::Vector3d bias_moment_world_{0.0, 0.0, 0.0};


  bool dry_run_{true};
  double delta_max_{0.05};
  int pub_decim_{10};
  std::atomic<int64_t> last_js_stamp_ns_{0};
  std::vector<double> last_enc_q_;
  bool first_cmd_checked_{false};
  int pub_count_{0};
  double loop_dt_{0.001};

  double torque_sign_{1.0};
  double deadband_force_{1.5};
  double deadband_moment_{0.5};

  // Helper: build a map from joint name �- DOF start index in the nrDof vector
  // RBDyn does not expose jointVelocityIndex(), we reconstruct it by walking
  // the joint list and accumulating DOF counts.
  std::map<std::string, int> buildJointDofMap(const rbd::MultiBody &mb)
  {
    std::map<std::string, int> dof_map;
    int dof_offset = 0;
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      const auto &j = mb.joint(i);
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
      // vectors to store encoder joint positions, velocities, and torques
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> enc_q(ref_order.size(), 0.0);
      std::vector<double> enc_alpha(ref_order.size(), 0.0);
      std::vector<double> enc_tau(ref_order.size(), 0.0);

      // Search for the joint in the received ROS JointState message
      for (size_t i = 0; i < ref_order.size(); ++i)
        for (size_t j = 0; j < msg->name.size(); ++j)
          if (msg->name[j] == ref_order[i])
          {
            enc_q[i] = msg->position[j];
            enc_alpha[i] = msg->velocity[j];
            enc_tau[i] = (j < msg->effort.size()) ? msg->effort[j] : 0.0;
            break;
          }

      // Send the reordered encoder to the robot controller
      gc_->setEncoderValues(gc_->robot().name(), enc_q);
      gc_->setEncoderVelocities(gc_->robot().name(), enc_alpha);
      gc_->setJointTorques(gc_->robot().name(), enc_tau);

      last_js_stamp_ns_ = this->now().nanoseconds();
      {
        std::lock_guard<std::mutex> lock(effort_mutex_);
        latest_efforts_ = enc_tau;
        last_enc_q_ = enc_q;
      }

      {
        std::lock_guard<std::mutex> lock(effort_mutex_);
        latest_efforts_ = enc_tau;
      }
      return;
    }

    std::lock_guard<std::mutex> lock(init_mutex_);
    if (initialized_)
      return;

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

    for (const auto &fs : gc_->robot().forceSensors())
    {
      mc_rtc::log::info("[KortexBridge] Registered Robot Force Sensor: '{}' on body: '{}'",
                        fs.name(), fs.parentBody());
    }

    // enable  mc_rtc controller execution
    gc_->running = true;

    for (size_t i = 0; i < ref_order.size(); ++i)
    {
      auto idx = gc_->robot().jointIndexByName(ref_order[i]);
      mc_rtc::log::info("[KortexBridge] Joint {} | init_q: {} | mbc.q: {}",
                        ref_order[i], init_q[i], gc_->robot().mbc().q[idx][0]);
    }

    initialized_ = true;
//    timer_ = this->create_wall_timer(1ms, std::bind(&KortexMcRtcBridge::controlLoop, this)); // 10ms
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(loop_dt_),
      std::bind(&KortexMcRtcBridge::controlLoop, this));

    mc_rtc::log::success("[KortexBridge] mc_rtc seeded. Control loop started!");
  }

  void controlLoop()
  {
    if (!initialized_)
      return;

    // Watchdog: stale joint states -> zero wrench, no commands
    const int64_t age_ns = this->now().nanoseconds() - last_js_stamp_ns_.load();
    const bool comms_ok = (age_ns < 100'000'000);  // 100 ms
    if (!comms_ok)
    {
      static int warn_count = 0;
      if (++warn_count % 1000 == 0)
        mc_rtc::log::error("[KortexBridge] /joint_states stale ({} ms) - holding", age_ns / 1'000'000);
    }

    const auto &robot = gc_->robot();
    const auto &mb = robot.mb(); // multi-body structure for mc_rtc

    // Build joint-name -> DOF-index map
    auto dof_map = buildJointDofMap(mb);

    ////// 1- Map measured joint torques - full nrDof vector
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

    ///// 2- Inverse dynamics to get gravity + Coriolis bias torques
    rbd::MultiBodyConfig mbc_id = robot.mbc();
    mbc_id.gravity = Eigen::Vector3d(0, 0, 9.81);
    for (auto &ad : mbc_id.alphaD)
      std::fill(ad.begin(), ad.end(), 0.0);

    rbd::InverseDynamics id(mb);
    id.inverseDynamics(mb, mbc_id);

    /////// 3- Extract bias torques into full Dof vector
    Eigen::VectorXd tau_bias = Eigen::VectorXd::Zero(mb.nrDof());
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      auto it = dof_map.find(mb.joint(i).name());
      // Convert joint-space torque to DOF-space torque vector
      if (it != dof_map.end() && !mbc_id.jointTorque[i].empty())
        tau_bias[it->second] = mbc_id.jointTorque[i][0];
    }

    ///// 4- Full Jacobian of end-effector frame
    rbd::Jacobian jac(mb, "tool_frame");
    Eigen::MatrixXd J_full = jac.jacobian(mb, robot.mbc());

    ///// 5- Extract only active joint components 
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
        // tau_ext_active[i] = tau_bias[col_idx] - tau_meas[col_idx]; /// Estimate external torque (bias - measured)
         tau_ext_active[i] = tau_bias[col_idx] - torque_sign_ * tau_meas[col_idx];
      }
    }

    ///// 6- Solve J_active - F_world = tau_ext_active
    Eigen::VectorXd F_world = J_active.transpose()
                                  .completeOrthogonalDecomposition()
                                  .solve(tau_ext_active);

    // Split wrench into moment and force components (world frame)
    Eigen::Vector3d moment_world(F_world[0], F_world[1], F_world[2]);
    Eigen::Vector3d force_world(F_world[3], F_world[4], F_world[5]);


    // delat at startup 
    static int startup_delay_ticks = 0;

    // Ignore wrench during initial stabilization period
    const int startup_ticks_max = static_cast<int>(3.0 / loop_dt_);

    if (startup_delay_ticks < startup_ticks_max) //3000
    {
      // During the 3-second startup delay, we keep command tracking active but force wrench to 0
      startup_delay_ticks++;
      force_world.setZero();
      moment_world.setZero();
    }
    else
    {
      // Dynamic World-Frame Tare (Zeroing) - runs AFTER the 3s delay is complete
      if (!tared_)
      {
        const int tare_ticks_max = static_cast<int>(0.5 / loop_dt_);   // 0.5 s of samples
        if (tare_ticks_ < tare_ticks_max) // it was 200 ticks 
        {
          bias_force_world_ += force_world;
          bias_moment_world_ += moment_world;
          tare_ticks_++;
        }
        else
        {
          bias_force_world_ /= tare_ticks_;
          bias_moment_world_ /= tare_ticks_;
          tared_ = true;
          mc_rtc::log::success("[KortexBridge] World-frame wrench tared successfully!");
          tared_ = true;
          mc_rtc::log::success("[KortexBridge] Tared. Bias force: ({:.2f},{:.2f},{:.2f}) N, norm={:.2f}",
                     bias_force_world_.x(), bias_force_world_.y(), bias_force_world_.z(),
                     bias_force_world_.norm());
        }
        force_world.setZero();
        moment_world.setZero();
      }
      else
      {
        // Subtract tared gravity offsets
        force_world -= bias_force_world_;
        moment_world -= bias_moment_world_;
      }
    }

    //// 7- Rotate the tared wrench from World Frame to Sensor Local Frame
    Eigen::Matrix3d R_world_sensor = robot.bodyPosW("tool_frame").rotation();

    Eigen::Vector3d moment_sensor = R_world_sensor * moment_world;
    Eigen::Vector3d force_sensor = R_world_sensor * force_world;

    // Filtering + deadbanding (only after taring)
    static Eigen::Vector3d filtered_force = Eigen::Vector3d::Zero();
    static Eigen::Vector3d filtered_moment = Eigen::Vector3d::Zero();
    if (tared_)
    {
      // 1. Accumulate the filter state normally (no resetting here!)
      const double tau_f = 0.05;                          // 50 ms time constant
      const double alpha_f = loop_dt_ / (loop_dt_ + tau_f);
      filtered_force  = (1.0 - alpha_f) * filtered_force  + alpha_f * force_sensor;
      filtered_moment = (1.0 - alpha_f) * filtered_moment + alpha_f * moment_sensor;
      //filtered_force = 0.90 * filtered_force + 0.10 * force_sensor; // 90% memory + 10% new
      //filtered_moment = 0.90 * filtered_moment + 0.10 * moment_sensor;

      // 2. temporary copies for output thresholding
      Eigen::Vector3d output_force = filtered_force;
      Eigen::Vector3d output_moment = filtered_moment;

      // 3. Apply Deadband on the temporary copies
      //if (output_force.norm() < 1.5)
      //{
       // output_force.setZero();
      //}
      //if (output_moment.norm() < 0.5)
      //{
       // output_moment.setZero();
      //}
      if (output_force.norm()  < deadband_force_)  output_force.setZero();
      if (output_moment.norm() < deadband_moment_) output_moment.setZero();

      // 4. Update the active sensor readings to send to mc_rtc
      force_sensor = output_force;
      moment_sensor = output_moment;
    }

    ///// 8- Inject estimated wrench into mc_rtc
    std::map<std::string, sva::ForceVecd> wrenches;

    double max_qd = 0.0;
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      if (mb.joint(i).dof() == 1) // only 1-DOF revolute joints
      {
        double qd = std::abs(robot.mbc().alpha[i][0]);
        if (qd > max_qd)
          max_qd = qd;
      }
    }

    // Gate: full wrench (1) below 0.05 rad/s, zero above 0.15 rad/s
    const double qd_low = 0.05;
    const double qd_high = 0.15;
    double gate = 1.0 - std::clamp((max_qd - qd_low) / (qd_high - qd_low), 0.0, 1.0);
    force_sensor *= gate;
    moment_sensor *= gate;

    wrenches["EEForceSensor"] = sva::ForceVecd(moment_sensor, force_sensor);

    // velocity-scaled wrench gate
    // When the arm is moving fast, the ID-based estimator is unreliable.
    // Scale the injected wrench toward zero as joint velocity increases.

    //double max_qd = 0.0;
    //for (int i = 0; i < mb.nrJoints(); ++i)
    //{
     // if (mb.joint(i).dof() == 1) // only 1-DOF revolute joints
      //{
       // double qd = std::abs(robot.mbc().alpha[i][0]);
        //if (qd > max_qd)
         // max_qd = qd;
      //}
    //}
    // Gate: full wrench (1) below 0.05 rad/s, zero above 0.15 rad/s
    //const double qd_low = 0.05;
    //const double qd_high = 0.15;
    //double gate = 1.0 - std::clamp((max_qd - qd_low) / (qd_high - qd_low), 0.0, 1.0);
    //force_sensor *= gate;
    //moment_sensor *= gate;

    if (!comms_ok) { force_sensor.setZero(); moment_sensor.setZero(); }
    //    wrenches["EEForceSensor"] = sva::ForceVecd(filtered_moment, filtered_force);
    gc_->setWrenches(wrenches);

    static int log_count = 0;
    if (++log_count % 500 == 0)
    {
      mc_rtc::log::info(
          "[KortexBridge] Est. wrench (Local) -- force: ({:.2f}, {:.2f}, {:.2f}) N  "
          "moment: ({:.2f}, {:.2f}, {:.2f}) Nm",
          force_sensor.x(), force_sensor.y(), force_sensor.z(),
          moment_sensor.x(), moment_sensor.y(), moment_sensor.z());
    }

    //// 9- Run controller and publish joint trajectory

    if (gc_->run() && comms_ok)
    {
      // --- Decimate: mc_rtc runs at 1 kHz, publish at ~100 Hz ---
      if (++pub_count_ % pub_decim_ != 0) return;

      static const std::vector<std::string> names =
          {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      // Snapshot encoders
      std::vector<double> enc_q;
      {
        std::lock_guard<std::mutex> lock(effort_mutex_);
        enc_q = last_enc_q_;
      }
      if (enc_q.size() < names.size()) return;

      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = names;
      trajectory_msgs::msg::JointTrajectoryPoint pt;

      bool sane = true;
      for (size_t k = 0; k < names.size(); ++k)
      {
        double q_cmd = 0.0, qd_cmd = 0.0;
        if (gc_->robot().hasJoint(names[k]))
        {
          auto idx = gc_->robot().jointIndexByName(names[k]);
          q_cmd  = gc_->robot().mbc().q[idx][0];
          qd_cmd = gc_->robot().mbc().alpha[idx][0];   // velocities for smooth JTC interp
        }

        // --- First-command sanity check ---
        if (!first_cmd_checked_ && std::abs(q_cmd - enc_q[k]) > 0.05)
        {
          mc_rtc::log::error(
              "[KortexBridge] FIRST CMD MISMATCH joint {} cmd={:.3f} enc={:.3f} - NOT publishing",
              names[k], q_cmd, enc_q[k]);
          sane = false;
        }

        // --- Per-cycle clamp around measured position ---
        q_cmd = std::clamp(q_cmd, enc_q[k] - delta_max_, enc_q[k] + delta_max_);

        pt.positions.push_back(q_cmd);
        pt.velocities.push_back(qd_cmd);
      }
      if (!sane) return;               // never publish a jumping first command
      first_cmd_checked_ = true;

      pt.time_from_start = rclcpp::Duration(0, 20'000'000); // 20 ms ≈ 2x publish period @100Hz
      traj.points.push_back(pt);

      if (dry_run_)
      {
        static int dr_count = 0;
        if (++dr_count % 100 == 0)
          mc_rtc::log::warning("[KortexBridge] DRY RUN - command not published");
        return;
      }
      pub_->publish(traj);
    }
   }
/*    if (gc_->run())
    {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      for (const auto &name : traj.joint_names)
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
      pt.time_from_start.nanosec = 20'000'000; // 20 ms - duration for 1 trajectory point
      traj.points.push_back(pt);
      pub_->publish(traj); 
    }
  } */

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

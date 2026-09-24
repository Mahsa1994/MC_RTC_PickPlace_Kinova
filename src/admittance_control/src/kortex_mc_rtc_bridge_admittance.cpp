// =============================================================================
//  kortex_mc_rtc_bridge_admittance.cpp
//
//  this bridge originally subscribed to a
//  "/EEForceSensor" WrenchStamped ROS topic that nothing on the real robot
//  publishes 
//  solved the identical problem by
//  computing its own gravity/inertia-compensated wrench estimate directly
//  from /joint_states' `effort` field 
//
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <RBDyn/ID.h>
#include <RBDyn/Jacobian.h>

#include <chrono>
#include <mutex>
#include <atomic>
#include <vector>
#include <map>
#include <thread>

using namespace std::chrono_literals;

class KortexMcRtcBridge : public rclcpp::Node
{
public:
  KortexMcRtcBridge() : Node("kortex_mc_rtc_bridge_admittance")
  {
    dry_run_    = this->declare_parameter("dry_run", true);
    delta_max_  = this->declare_parameter("delta_max", 0.05);
    model_real_gate_ = this->declare_parameter("model_real_gate", 0.05); // rad; hard publish gate, see run()
    publish_rate_ = this->declare_parameter("publish_rate", 200.0); // Hz; capped at the control rate

    torque_sign_    = this->declare_parameter("torque_sign", 1.0);   // flip to -1.0 if inverted on real

    // Tool payload not present in the mc_kinova URDF (Robotiq 2F-85 + coupling),
    // modelled as a point mass at payload_com expressed in bracelet_link. The
    // flange is at z=-0.0615 in that frame; the gripper COM sits ~6 cm beyond it.
    payload_mass_ = this->declare_parameter("payload_mass", 0.9);
    payload_com_  = this->declare_parameter("payload_com", std::vector<double>{0.0, 0.0, -0.12});
    payload_body_ = this->declare_parameter("payload_body", std::string("bracelet_link"));

    // Per-joint torque deadband (Nm). Reusing the old Cartesian moment
    // deadband's value as the starting point for the joint-space version -
    // same units (Nm), not yet re-tuned per joint.
    deadband_torque_ = this->declare_parameter("deadband_torque", 0.5);

    // Velocity gate for the wrench estimate (rad/s). Raised from the old
    // 0.05/0.15 defaults now that inertial torque is compensated (see the
    // alphaD estimation below) - re-validate against real push-while-moving
    // tests before trusting these numbers on hardware.
    qd_gate_low_  = this->declare_parameter("qd_gate_low", 0.05);
    qd_gate_high_ = this->declare_parameter("qd_gate_high", 0.3);

    // Low-pass time constant for the finite-differenced joint-acceleration
    // estimate used to compensate inertial torque in the bias computation.
    accel_filter_tau_ = this->declare_parameter("accel_filter_tau", 0.1);

    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 1);
    tau_pub_     = this->create_publisher<std_msgs::msg::Float64MultiArray>("/admittance/tau_ext", 10);
    tau_raw_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/admittance/tau_ext_raw", 10);

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

  // Software-level per-joint torque tare (gravity/inertia-residual bias),
  // sized to refJointOrder() once initialized.
  bool tau_tared_{false};
  int tau_tare_ticks_{0};
  Eigen::VectorXd bias_tau_;


  bool dry_run_{true};
  double delta_max_{0.05};
  double model_real_gate_{0.05};
  double publish_rate_{200.0};
  int pub_decim_{1};
  double payload_mass_{0.0};
  std::vector<double> payload_com_;
  std::string payload_body_;
  std::unique_ptr<rbd::Jacobian> payload_jac_;
  Eigen::MatrixXd payload_jac_full_;
  std::chrono::steady_clock::time_point last_loop_tp_;
  bool has_last_loop_tp_{false};
  double loop_period_sum_{0.0};
  double loop_period_max_{0.0};
  int loop_period_n_{0};
  std::atomic<int64_t> last_js_stamp_ns_{0};
  std::vector<double> last_enc_q_;
  bool first_cmd_checked_{false};
  int pub_count_{0};
  // Always set from gc_->timestep() once the controller is initialized (see
  // jointStateCallback) - must match the controller's configured Timestep
  // exactly, since the QP/trajectory tasks integrate assuming this many
  // seconds pass per gc_->run() call regardless of real wall-clock time.
  // Calling run() faster than this makes the internal model race ahead of
  // real time (e.g. a "10s" trajectory finishes in 2s of wall time), which
  // is what produced the runaway QP divergence and eventual segfault seen
  // on 2026-09-18.
  double loop_dt_{0.005};

  double torque_sign_{1.0};
  double deadband_torque_{0.5};

  double qd_gate_low_{0.05};
  double qd_gate_high_{0.3};
  double accel_filter_tau_{0.1};

  // Finite-differenced, filtered joint acceleration (DOF-space) fed into the
  // inverse-dynamics bias so genuine inertial torque isn't misread as contact.
  Eigen::VectorXd filtered_alphaD_;
  Eigen::VectorXd prev_alpha_dof_;
  bool has_prev_alpha_{false};

  // Helper: build a map from joint name -> DOF start index in the nrDof vector
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

  // Soft (continuous) deadband: shrink the value's magnitude by `db` instead
  // of zeroing it below a hard threshold. A hard cutoff is discontinuous
  // exactly where guiding happens: 0.49 Nm gives nothing, 0.51 Nm gives the
  // full 0.51 Nm. That cliff chatters - push past the threshold, the joint
  // moves, the measured torque falls back under it, it snaps to zero, the
  // joint stops, push again. Soft thresholding rejects sub-deadband noise
  // exactly as before (output is still identically zero below `db`) but has
  // no step at the crossing.
  static double softDeadband(double v, double db)
  {
    const double n = std::abs(v);
    if (n <= db)
      return 0.0;
    return v * ((n - db) / n);
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

    // Per-joint external torque estimate, published each control tick below
    // and consumed by HandGuideState's joint-space admittance law.
    gc_->controller().datastore().make<Eigen::VectorXd>(
        "KHG::tau_ext", Eigen::VectorXd::Zero(static_cast<int>(ref_order.size())));

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

    // Drive the control loop at exactly the controller's configured
    // Timestep. The QP/trajectory tasks assume this many seconds pass per
    // gc_->run() call regardless of real elapsed time, so any mismatch here
    // makes the internal model race ahead of (or lag behind) real time.
    loop_dt_ = gc_->timestep();
    pub_decim_ = std::max(1, static_cast<int>(std::lround(1.0 / (loop_dt_ * publish_rate_))));
    mc_rtc::log::info("[KortexBridge] Control loop period set to controller timestep: {} s; publishing every {} tick(s) ({:.0f} Hz)",
                      loop_dt_, pub_decim_, 1.0 / (loop_dt_ * pub_decim_));

    if (payload_mass_ > 0.0)
    {
      if (gc_->robot().hasBody(payload_body_) && payload_com_.size() == 3)
      {
        payload_jac_ = std::make_unique<rbd::Jacobian>(
            gc_->robot().mb(), payload_body_,
            Eigen::Vector3d(payload_com_[0], payload_com_[1], payload_com_[2]));
        payload_jac_full_ = Eigen::MatrixXd::Zero(6, gc_->robot().mb().nrDof());
        mc_rtc::log::info("[KortexBridge] Payload compensation: {:.3f} kg at [{:.3f}, {:.3f}, {:.3f}] in {}",
                          payload_mass_, payload_com_[0], payload_com_[1], payload_com_[2], payload_body_);
      }
      else
      {
        mc_rtc::log::error("[KortexBridge] payload_body '{}' not in model or payload_com not 3 entries - payload compensation DISABLED",
                           payload_body_);
        payload_mass_ = 0.0;
      }
    }

    initialized_ = true;
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(loop_dt_),
      std::bind(&KortexMcRtcBridge::controlLoop, this));

    mc_rtc::log::success("[KortexBridge] mc_rtc seeded. Control loop started!");
  }

  void controlLoop()
  {
    if (!initialized_)
      return;

    // Loop-rate diagnostics: the QP integrates assuming loop_dt_ per call, so
    // a timer that can't keep up silently slows the internal model down.
    {
      const auto now_tp = std::chrono::steady_clock::now();
      if (has_last_loop_tp_)
      {
        const double p = std::chrono::duration<double>(now_tp - last_loop_tp_).count();
        loop_period_sum_ += p;
        loop_period_max_ = std::max(loop_period_max_, p);
        if (++loop_period_n_ >= 1000)
        {
          const double mean = loop_period_sum_ / loop_period_n_;
          if (mean > 1.2 * loop_dt_ || loop_period_max_ > 5.0 * loop_dt_)
            mc_rtc::log::warning("[KortexBridge] control loop period mean {:.2f} ms / max {:.2f} ms (target {:.2f} ms)",
                                 1e3 * mean, 1e3 * loop_period_max_, 1e3 * loop_dt_);
          loop_period_sum_ = 0.0; loop_period_max_ = 0.0; loop_period_n_ = 0;
        }
      }
      last_loop_tp_ = now_tp;
      has_last_loop_tp_ = true;
    }

    // Watchdog: stale joint states -> zero wrench, no commands
    const int64_t age_ns = this->now().nanoseconds() - last_js_stamp_ns_.load();
    const bool comms_ok = (age_ns < 100'000'000);  // 100 ms
    if (!comms_ok)
    {
      static int warn_count = 0;
      if (++warn_count % 1000 == 0)
        mc_rtc::log::error("[KortexBridge] /joint_states stale ({} ms) - holding", age_ns / 1'000'000);
    }

    // The wrench estimate below (inverse dynamics, Jacobian, sensor-frame
    // rotation, velocity gate) must be evaluated at the configuration the arm
    // is ACTUALLY in, i.e. the encoder-observed realRobot - not gc_->robot(),
    // which is the QP's own open-loop integration state. Using the control
    // robot made gravity compensation diverge exactly when it mattered: once
    // the model led the real arm, tau_bias was computed at the wrong q, the
    // residual showed up as a phantom contact wrench, the admittance
    // integrated it, and the model ran away further. That positive feedback
    // is what latched the model_real_gate at 0.10 rad on 2026-09-03.
    const auto &robot = gc_->realRobot();
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

    ///// 2- Inverse dynamics to get gravity + Coriolis + inertial bias torques
    //
    // alphaD used to be forced to zero here, which implicitly assumes the arm
    // is never accelerating. In reality it almost always is at least a
    // little (even small corrective jitter while "holding still"), so the
    // real inertial torque M(q)*qddot was silently misattributed to external
    // contact - a likely cause of jerky motion with nobody touching the arm.
    // Estimate qddot via a filtered finite-difference of encoder velocity so
    // it's accounted for in the bias instead.
    Eigen::VectorXd alpha_dof = Eigen::VectorXd::Zero(mb.nrDof());
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      if (mb.joint(i).dof() == 1)
      {
        auto it = dof_map.find(mb.joint(i).name());
        if (it != dof_map.end())
          alpha_dof[it->second] = robot.mbc().alpha[i][0];
      }
    }

    if (filtered_alphaD_.size() != mb.nrDof())
      filtered_alphaD_ = Eigen::VectorXd::Zero(mb.nrDof());

    if (has_prev_alpha_)
    {
      Eigen::VectorXd raw_alphaD = (alpha_dof - prev_alpha_dof_) / loop_dt_;
      const double alpha_f_acc = loop_dt_ / (loop_dt_ + accel_filter_tau_);
      filtered_alphaD_ = (1.0 - alpha_f_acc) * filtered_alphaD_ + alpha_f_acc * raw_alphaD;
    }
    prev_alpha_dof_  = alpha_dof;
    has_prev_alpha_  = true;

    rbd::MultiBodyConfig mbc_id = robot.mbc();
    mbc_id.gravity = Eigen::Vector3d(0, 0, 9.81);
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      auto it = dof_map.find(mb.joint(i).name());
      if (it != dof_map.end() && !mbc_id.alphaD[i].empty())
        mbc_id.alphaD[i][0] = filtered_alphaD_[it->second];
    }

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

    // Payload gravity: motors must supply J_lin^T * (0, 0, m*g) to hold the
    // point mass, same sign convention as RBDyn's ID with gravity = (0,0,9.81).
    if (payload_jac_)
    {
      const Eigen::MatrixXd & jac = payload_jac_->jacobian(mb, robot.mbc());
      payload_jac_->fullJacobian(mb, jac, payload_jac_full_);
      const Eigen::Vector3d f(0.0, 0.0, payload_mass_ * 9.81);
      tau_bias += payload_jac_full_.bottomRows<3>().transpose() * f;
    }

    ///// 4- Per-joint external torque residual (no Jacobian/Cartesian solve -
    // this is the final signal now, not an intermediate for a wrench).
    auto ref_order = robot.refJointOrder();
    Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(ref_order.size());
    for (size_t i = 0; i < ref_order.size(); ++i)
    {
      auto it = dof_map.find(ref_order[i]);
      if (it != dof_map.end())
        tau_ext[i] = tau_bias[it->second] - torque_sign_ * tau_meas[it->second];
    }

    if (bias_tau_.size() != static_cast<int>(ref_order.size()))
      bias_tau_ = Eigen::VectorXd::Zero(ref_order.size());

    // delay at startup
    static int startup_delay_ticks = 0;

    // Ignore torque estimate during initial stabilization period
    const int startup_ticks_max = static_cast<int>(3.0 / loop_dt_);

    if (startup_delay_ticks < startup_ticks_max) //3000
    {
      // During the 3-second startup delay, we keep command tracking active but force the estimate to 0
      startup_delay_ticks++;
      tau_ext.setZero();
    }
    else
    {
      // Dynamic per-joint tare (zeroing) - runs AFTER the 3s delay is complete
      if (!tau_tared_)
      {
        const int tare_ticks_max = static_cast<int>(0.5 / loop_dt_);   // 0.5 s of samples
        if (tau_tare_ticks_ < tare_ticks_max)
        {
          bias_tau_ += tau_ext;
          tau_tare_ticks_++;
        }
        else
        {
          bias_tau_ /= tau_tare_ticks_;
          tau_tared_ = true;
          mc_rtc::log::success("[KortexBridge] Per-joint torque tared successfully!");
        }
        tau_ext.setZero();
      }
      else
      {
        // Subtract tared gravity/inertia-model residual bias
        tau_ext -= bias_tau_;
      }
    }

    // Raw (Local, per-joint) torque, post-tare but before the 50ms low-pass
    // filter and the deadband - kept aside purely so the periodic log below
    // can show what the estimator actually computed vs. what survived
    // filtering/deadbanding.
    const Eigen::VectorXd raw_tau_ext = tau_ext;

    // Filtering + deadbanding (only after taring)
    static Eigen::VectorXd filtered_tau = Eigen::VectorXd::Zero(ref_order.size());
    if (filtered_tau.size() != static_cast<int>(ref_order.size()))
      filtered_tau = Eigen::VectorXd::Zero(ref_order.size());
    if (tau_tared_)
    {
      // 1. Accumulate the filter state normally (no resetting here!)
      const double tau_f = 0.05;                          // 50 ms time constant
      const double alpha_f = loop_dt_ / (loop_dt_ + tau_f);
      filtered_tau = (1.0 - alpha_f) * filtered_tau + alpha_f * tau_ext;

      // 2. Apply the per-joint soft deadband on a temporary copy (see softDeadband)
      Eigen::VectorXd output_tau = filtered_tau;
      for (int i = 0; i < output_tau.size(); ++i)
        output_tau[i] = softDeadband(output_tau[i], deadband_torque_);

      // 3. Update the active reading to send to mc_rtc
      tau_ext = output_tau;
    }

    ///// 5- Per-joint velocity gate: each joint's own motion suppresses its
    // own torque reading (rejects inertial jitter as "contact"), rather than
    // any joint's motion suppressing the whole arm's compliance as the old
    // Cartesian version did.
    for (size_t i = 0; i < ref_order.size(); ++i)
    {
      auto idx = robot.jointIndexByName(ref_order[i]);
      double qd = std::abs(robot.mbc().alpha[idx][0]);
      double gate = 1.0 - std::clamp((qd - qd_gate_low_) / (qd_gate_high_ - qd_gate_low_), 0.0, 1.0);
      tau_ext[i] *= gate;
    }

    // Must come BEFORE the estimate is published - zeroing afterwards was
    // dead code in the old version, so a comms dropout still injected the
    // last estimate into the controller.
    if (!comms_ok) { tau_ext.setZero(); }

    ///// 6- Publish the per-joint external torque estimate to the datastore
    // for HandGuideState's joint-space admittance law.
    gc_->controller().datastore().assign<Eigen::VectorXd>("KHG::tau_ext", tau_ext);

    {
      std_msgs::msg::Float64MultiArray m;
      m.data.assign(tau_ext.data(), tau_ext.data() + tau_ext.size());
      tau_pub_->publish(m);
      m.data.assign(raw_tau_ext.data(), raw_tau_ext.data() + raw_tau_ext.size());
      tau_raw_pub_->publish(m);
    }

    static int log_count = 0;
    if (++log_count % 500 == 0)
    {
      auto fmtVec = [](const Eigen::VectorXd & v)
      {
        std::string s;
        for (int i = 0; i < v.size(); ++i)
          s += fmt::format("{}{:.3f}", i == 0 ? "" : ", ", v[i]);
        return s;
      };
      mc_rtc::log::info("[KortexBridge] RAW tau_ext (Nm, pre-filter/deadband): [{}]", fmtVec(raw_tau_ext));
      mc_rtc::log::info("[KortexBridge] Est. tau_ext (Nm): [{}]", fmtVec(tau_ext));
    }

    // Model-vs-real divergence: the QP-solved control robot (robot.mbc(),
    // used above for gravity/inertial compensation, and gc_->robot() below
    // for the published command) integrates open-loop from the solver's
    // own accelerations and is NEVER resynced to the real, encoder-observed
    // arm. Computed every tick and used as a hard publish gate below
    // (model_real_gate_) - see the impedance bridge's history for why
    // (three live E-stops before this existed).
    double model_real_dev = 0.0;
    std::string model_real_worst_joint;
    double model_real_worst_model_q = 0.0;
    double model_real_worst_enc_q   = 0.0;
    {
      std::lock_guard<std::mutex> lock(effort_mutex_);
      // NB: `robot` above is the realRobot; this check is specifically about
      // the CONTROL robot drifting away from it, so read q from gc_->robot().
      const auto &ctl_robot = gc_->robot();
      auto ref_order = ctl_robot.refJointOrder();
      for (size_t i = 0; i < ref_order.size() && i < last_enc_q_.size(); ++i)
      {
        auto idx = ctl_robot.jointIndexByName(ref_order[i]);
        double model_q = ctl_robot.mbc().q[idx][0];
        double dev = std::abs(model_q - last_enc_q_[i]);
        if (dev > model_real_dev)
        {
          model_real_dev = dev;
          model_real_worst_joint = ref_order[i];
          model_real_worst_model_q = model_q;
          model_real_worst_enc_q   = last_enc_q_[i];
        }
      }
      if (log_count % 500 == 0)
        mc_rtc::log::info(
            "[KortexBridge] model-vs-real max joint deviation: {:.4f} rad on '{}' "
            "(model={:.4f} rad, real={:.4f} rad)",
            model_real_dev, model_real_worst_joint, model_real_worst_model_q, model_real_worst_enc_q);
    }


    //// 9- Run controller and publish joint trajectory

    if (gc_->run() && comms_ok)
    {
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

      // 2x the publish period so the JTC always has a segment to interpolate into
      pt.time_from_start = rclcpp::Duration::from_seconds(2.0 * loop_dt_ * pub_decim_);
      traj.points.push_back(pt);

      if (dry_run_)
      {
        static int dr_count = 0;
        if (++dr_count % 100 == 0)
          mc_rtc::log::warning("[KortexBridge] DRY RUN - command not published");
        return;
      }

      // --- SAFETY GATE: model-vs-real divergence (computed above, every tick) ---
      if (model_real_dev > model_real_gate_)
      {
        static int gate_warn_count = 0;
        if (++gate_warn_count % 20 == 0)
          mc_rtc::log::error(
              "[KortexBridge] SAFETY GATE: model-vs-real deviation {:.4f} rad on '{}' exceeds "
              "model_real_gate ({:.4f} rad) - NOT publishing until this clears",
              model_real_dev, model_real_worst_joint, model_real_gate_);
        return;
      }

      pub_->publish(traj);
    }
   }

  std::shared_ptr<mc_control::MCGlobalController> gc_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_raw_pub_;
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

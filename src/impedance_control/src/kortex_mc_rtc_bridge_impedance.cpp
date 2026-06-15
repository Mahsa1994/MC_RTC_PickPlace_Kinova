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

private:
  std::mutex init_mutex_;
  std::atomic<bool> initialized_{false};

  std::vector<double> latest_efforts_;
  std::mutex effort_mutex_;

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

    const auto & robot = gc_->robot();
    const auto & mb    = robot.mb();

    // ── Build joint-name → DOF-index map (replaces missing jointVelocityIndex)
    auto dof_map = buildJointDofMap(mb);

    // ── 1. Map measured joint torques → full nrDof vector
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

    // ── 2. Inverse dynamics to get gravity + Coriolis bias torques
    rbd::MultiBodyConfig mbc_id = robot.mbc();
    mbc_id.gravity = Eigen::Vector3d(0, 0, 9.81); // must be set — zeroed by default
    for (auto & ad : mbc_id.alphaD)
      std::fill(ad.begin(), ad.end(), 0.0);        // zero acceleration → pure bias

    rbd::InverseDynamics id(mb);
    id.inverseDynamics(mb, mbc_id);

    // ── 3. Extract bias torques into nrDof vector
    Eigen::VectorXd tau_bias = Eigen::VectorXd::Zero(mb.nrDof());
    for (int i = 0; i < mb.nrJoints(); ++i)
    {
      auto it = dof_map.find(mb.joint(i).name());
      if (it != dof_map.end() && !mbc_id.jointTorque[i].empty())
        tau_bias[it->second] = mbc_id.jointTorque[i][0];
    }

    // ── 4. Jacobian of end-effector frame
    rbd::Jacobian jac(mb, "bracelet_link");
    Eigen::MatrixXd J = jac.jacobian(mb, robot.mbc()); // 6 × nrDof

    // ── 5. Solve  Jᵀ · F_ext = tau_meas - tau_bias
    //    (external torque = measured − gravity/Coriolis bias)
    Eigen::VectorXd tau_ext = tau_meas - tau_bias;
    // completeOrthogonalDecomposition handles the redundancy of a 6-DOF arm gracefully
    Eigen::VectorXd F_est = J.transpose()
                              .completeOrthogonalDecomposition()
                              .solve(tau_ext); // → [moment(3); force(3)]

    // ── 6. Inject estimated wrench into mc_rtc
    // sva::ForceVecd layout: (couple/moment, force)
    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches["EEForceSensor"] = sva::ForceVecd(
        Eigen::Vector3d(F_est[0], F_est[1], F_est[2]),  // moment (Nm)
        Eigen::Vector3d(F_est[3], F_est[4], F_est[5])   // force  (N)
    );
    gc_->setWrenches(wrenches);

    static int log_count = 0;
    if (++log_count % 500 == 0)
      mc_rtc::log::info(
          "[KortexBridge] Est. wrench — force: ({:.2f}, {:.2f}, {:.2f}) N  "
          "moment: ({:.2f}, {:.2f}, {:.2f}) Nm",
          F_est[3], F_est[4], F_est[5],
          F_est[0], F_est[1], F_est[2]);

    // ── 7. Run controller and publish joint trajectory
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

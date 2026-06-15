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

    // Initialize effort tracking
    auto ref_order = gc_->robot().refJointOrder();
    latest_efforts_.resize(ref_order.size(), 0.0);

    mc_rtc::log::info("[KortexBridge] Waiting for first /joint_states...");
    mc_rtc::log::info("[KortexBridge] Operating in joint torque estimation mode.");
  }

private:
  std::mutex init_mutex_;
  std::atomic<bool> initialized_{false};

  // Effort (torque) storage
  std::vector<double> latest_efforts_;
  std::mutex effort_mutex_;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (initialized_)
    {
      auto ref_order = gc_->robot().refJointOrder();
      std::vector<double> enc_q(ref_order.size(), 0.0);
      std::vector<double> enc_alpha(ref_order.size(), 0.0);
      std::vector<double> enc_tau(ref_order.size(), 0.0);

      for (size_t i = 0; i < ref_order.size(); ++i)
      {
        for (size_t j = 0; j < msg->name.size(); ++j)
        {
          if (msg->name[j] == ref_order[i])
          {
            enc_q[i]     = msg->position[j];
            enc_alpha[i] = msg->velocity[j];
            enc_tau[i]   = msg->effort[j];
            break;
          }
        }
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

    // 1. Construct measured torque vector aligned with MultiBody DOFs
    Eigen::VectorXd tau_meas = Eigen::VectorXd::Zero(robot.mb().nrDof());
    {
      std::lock_guard<std::mutex> lock(effort_mutex_);
      auto ref_order = robot.refJointOrder();
      for (size_t i = 0; i < ref_order.size(); ++i)
      {
        if (robot.hasJoint(ref_order[i]))
        {
          auto joint_idx = robot.jointIndexByName(ref_order[i]);
          auto dof_idx = robot.mb().jointVelocityIndex(joint_idx);
          if (dof_idx >= 0 && dof_idx < tau_meas.size())
          {
            tau_meas[dof_idx] = latest_efforts_[i];
          }
        }
      }
    }

    // 2. Compute bias torques (gravity + Coriolis) using Inverse Dynamics
    // Create a copy of mbc where joint accelerations (alphaD) are set to zero
    rbd::MultiBodyConfig mbc_id = robot.mbc();
    for (auto & ad : mbc_id.alphaD)
    {
      std::fill(ad.begin(), ad.end(), 0.0);
    }
    
    rbd::InverseDynamics id(robot.mb());
    rbd::MultiBodyConfig mbc_id = robot.mbc();
    mbc_id.gravity = Eigen::Vector3d(0, 0, 9.81); // <-- add this line
    for (auto & ad : mbc_id.alphaD)
       std::fill(ad.begin(), ad.end(), 0.0);
    id.inverseDynamics(robot.mb(), mbc_id);
    
    // Extract tau_bias from mbc_id.jointTorque
    Eigen::VectorXd tau_bias = Eigen::VectorXd::Zero(robot.mb().nrDof());
    for (int i = 0; i < robot.mb().nrJoints(); ++i)
    {
      auto dof_idx = robot.mb().jointVelocityIndex(i);
      if (dof_idx >= 0 && dof_idx < tau_bias.size())
      {
        if (!mbc_id.jointTorque[i].empty())
        {
          tau_bias[dof_idx] = mbc_id.jointTorque[i][0];
        }
      }
    }

    // 3. Compute Jacobian of the end-effector "bracelet_link"
    rbd::Jacobian jac(robot.mb(), "bracelet_link");
    Eigen::MatrixXd J_body = jac.jacobian(robot.mb(), robot.mbc());
    
    // 4. Solve J_body^T * F_ext = tau_bias - tau_meas
    // Using completeOrthogonalDecomposition is safe near singular configurations
    Eigen::MatrixXd JT = J_body.transpose();
    Eigen::VectorXd tau_ext = tau_bias - tau_meas;
    Eigen::VectorXd F_est_vec = JT.completeOrthogonalDecomposition().solve(tau_ext);

    // 5. Inject estimated wrench into mc_rtc under the "EEForceSensor" name
    std::map<std::string, sva::ForceVecd> wrenches;
    wrenches["EEForceSensor"] = sva::ForceVecd(
        Eigen::Vector3d(F_est_vec[0], F_est_vec[1], F_est_vec[2]), // estimated torque
        Eigen::Vector3d(F_est_vec[3], F_est_vec[4], F_est_vec[5])  // estimated force
    );
    gc_->setWrenches(wrenches);

    static int log_count = 0;
    if (++log_count % 500 == 0)
    {
      mc_rtc::log::info("[KortexBridge] Est. Wrench: force=({},{},{}) torque=({},{},{})",
                        F_est_vec[3], F_est_vec[4], F_est_vec[5],
                        F_est_vec[0], F_est_vec[1], F_est_vec[2]);
    }

    // 6. Run the controller step
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
        {
          pt.positions.push_back(0.0);
        }
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

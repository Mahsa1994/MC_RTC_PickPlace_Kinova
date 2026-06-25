#include "ImpedanceHoldState.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/ImpedanceTask.h> 

void ImpedanceHoldState::configure(const mc_rtc::Configuration & config)
{
  config_ = config; 
  if (config.has("stiffness"))     task_stiffness_ = config("stiffness");
  if (config.has("damping"))       task_damping_   = config("damping");
  if (config.has("weight"))        task_weight_    = config("weight");
}

void ImpedanceHoldState::start(mc_control::fsm::Controller & ctl)
{
  dt_ = ctl.timeStep;

  // 1. Capture current EE pose as the hold target using tool_frame
  X_0_target_ = ctl.robot().frame("tool_frame").position();

  // 2. Create active ImpedanceTask on tool_frame
  task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(
      ctl.robot().frame("tool_frame"),
      task_stiffness_,
      task_weight_);

  // Set the cutoff period to smooth out PID chatter
  task_->cutoffPeriod(0.3);

  // 3. Set kinematic solver tracking parameters
  task_->stiffness(task_stiffness_);
  task_->damping(task_damping_);
  task_->targetPose(X_0_target_); 

  // 4. Get a reference to the task's existing gains to modify in-place
  auto & gains = task_->gains();

  // Set mass parameters
  gains.mass().linear(Eigen::Vector3d(3.0, 3.0, 3.0));
  gains.mass().angular(Eigen::Vector3d(2.0, 2.0, 2.0));

  // Set spring (stiffness) parameters
  gains.spring().linear(Eigen::Vector3d(10.0, 10.0, 10.0));
  gains.spring().angular(Eigen::Vector3d(8.0, 8.0, 8.0));

  // Set damper (damping) parameters
  gains.damper().linear(Eigen::Vector3d(25.0, 25.0, 25.0));
  gains.damper().angular(Eigen::Vector3d(15.0, 15.0, 15.0));

  // Set wrench threshold (deadband) parameters
  gains.wrench().linear(Eigen::Vector3d(0.0, 0.0, 0.0));       // 3.0 N force threshold
  gains.wrench().angular(Eigen::Vector3d(0.0, 0.0, 0.0));      // 1.0 Nm torque threshold

  ctl.solver().addTask(task_);

  mc_rtc::log::success("[ImpedanceHoldState] Compliant active hold enabled via C++");
}

bool ImpedanceHoldState::run(mc_control::fsm::Controller & ctl)
{
  static int log_count = 0;
  if (++log_count % 200 == 0)
  {
    const sva::PTransformd X_compliance = task_->compliancePose();
    const sva::PTransformd X_actual = ctl.robot().frame("tool_frame").position();
    Eigen::Vector3d lin_err = X_actual.translation() - X_compliance.translation();

    Eigen::Vector3d track_err = X_actual.translation() - X_compliance.translation();
    Eigen::Vector3d compliance_deflection = X_compliance.translation() - X_0_target_.translation();

    const sva::ForceVecd wrench = task_->measuredWrench();

/*    mc_rtc::log::info("[ImpedanceHoldState] Force: ({:.2f}, {:.2f}, {:.2f}) N | "
                      "Err to Compliance Target: ({:.4f},{:.4f},{:.4f})m  "
                      "dist={:.4f}m",
                      wrench.force().x(), wrench.force().y(), wrench.force().z(),
                      lin_err.x(), lin_err.y(), lin_err.z(),
                      lin_err.norm()); */
    mc_rtc::log::info("[ImpedanceHoldState] Force: ({:.1f}, {:.1f}, {:.1f}) N | "
                      "Deflection: ({:.3f},{:.3f},{:.3f})m (dist={:.3f}m) | "
                      "Track Error: {:.4f}m",
                      wrench.force().x(), wrench.force().y(), wrench.force().z(),
                      compliance_deflection.x(), compliance_deflection.y(), compliance_deflection.z(),
                      compliance_deflection.norm(),
                      track_err.norm());
  }

  return false;
}

void ImpedanceHoldState::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(task_);
  mc_rtc::log::info("[ImpedanceHoldState] Task removed");
}

EXPORT_SINGLE_STATE("KI::ImpedanceHoldState", ImpedanceHoldState)

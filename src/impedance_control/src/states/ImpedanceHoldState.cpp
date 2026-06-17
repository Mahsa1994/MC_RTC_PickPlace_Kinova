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

  // 2. Create active ImpedanceTask on tool_frame (will find EEForceSensor natively!)
  task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(
      ctl.robot().frame("tool_frame"),
      task_stiffness_,
      task_weight_);

  // 3. Configure the impedance task
  task_->load(ctl.solver(), config_);

  // Set the cutoff period to smooth out PID chatter
  task_->cutoffPeriod(0.3);

  // 4. Set kinematic tracking parameters
  task_->stiffness(task_stiffness_);
  task_->damping(task_damping_);
  task_->targetPose(X_0_target_); 

  ctl.solver().addTask(task_);

  mc_rtc::log::success("[ImpedanceHoldState] Compliant active hold active");
  mc_rtc::log::info("[ImpedanceHoldState] tracking_stiffness={} tracking_damping={} weight={}",
    task_stiffness_, task_damping_, task_weight_);
}

bool ImpedanceHoldState::run(mc_control::fsm::Controller & ctl)
{
  static int log_count = 0;
  if (++log_count % 200 == 0)
  {
    const sva::PTransformd X_compliance = task_->compliancePose();
    const sva::PTransformd X_actual = ctl.robot().frame("tool_frame").position();
    Eigen::Vector3d lin_err = X_actual.translation() - X_compliance.translation();
    
    const sva::ForceVecd wrench = task_->measuredWrench();
    
    mc_rtc::log::info("[ImpedanceHoldState] Force: ({:.2f}, {:.2f}, {:.2f}) N | "
                      "Err to Compliance Target: ({:.4f},{:.4f},{:.4f})m  "
                      "dist={:.4f}m",
                      wrench.force().x(), wrench.force().y(), wrench.force().z(),
                      lin_err.x(), lin_err.y(), lin_err.z(),
                      lin_err.norm());
  }

  return false;
}

void ImpedanceHoldState::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(task_);
  mc_rtc::log::info("[ImpedanceHoldState] Task removed");
}

EXPORT_SINGLE_STATE("KI::ImpedanceHoldState", ImpedanceHoldState)

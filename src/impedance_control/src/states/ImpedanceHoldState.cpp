#include "ImpedanceHoldState.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/ImpedanceTask.h> // Include active ImpedanceTask

void ImpedanceHoldState::configure(const mc_rtc::Configuration & config)
{
  config_ = config; // Save the configuration block to pass to the task later
  if (config.has("stiffness"))     task_stiffness_ = config("stiffness");
  if (config.has("damping"))       task_damping_   = config("damping");
  if (config.has("weight"))        task_weight_    = config("weight");
}

void ImpedanceHoldState::start(mc_control::fsm::Controller & ctl)
{
  dt_ = ctl.timeStep;

  // 1. Register the virtual force sensor on the control robot.
  // This acts as the bridge receiver so setWrenches() doesn't get ignored.
  if (!ctl.robot().hasForceSensor("EEForceSensor"))
  {
    mc_rtc::log::info("[ImpedanceHoldState] Registering virtual force sensor 'EEForceSensor' on 'bracelet_link'");
    mc_rbdyn::ForceSensor virtual_fs("EEForceSensor", "bracelet_link", sva::PTransformd::Identity());
    ctl.robot().addForceSensor(virtual_fs);
  }

  // 2. Capture current EE pose as the hold target (the path point)
  X_0_target_ = ctl.robot().frame("bracelet_link").position();

  // 3. Create active ImpedanceTask (instead of kinematic TransformTask)
  task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(
      ctl.robot().frame("bracelet_link"),
      task_stiffness_,
      task_weight_);

  // 4. Configure the impedance task (loads the 'gains' block from your YAML)
  // MetaTasks use load() to ingest mc_rtc::Configuration
  task_->load(ctl.solver(), config_);
  
  task_->cutoffPeriod(0.3);

  // 5. Set kinematic tracking parameters
  task_->stiffness(task_stiffness_);
  task_->damping(task_damping_);
  
  // Set the target using targetPose() because target() is private in ImpedanceTask
  task_->targetPose(X_0_target_); 

  ctl.solver().addTask(task_);

  mc_rtc::log::success("[ImpedanceHoldState] Compliant active hold active");
  mc_rtc::log::info("[ImpedanceHoldState] tracking_stiffness={} tracking_damping={} weight={}",
    task_stiffness_, task_damping_, task_weight_);
}

bool ImpedanceHoldState::run(mc_control::fsm::Controller & ctl)
{
  // Periodic logging of the compliance pose tracking error
  static int log_count = 0;
  if (++log_count % 200 == 0)
  {
    // compliancePose() is the virtual compliant target updated by force feedback
    const sva::PTransformd X_compliance = task_->compliancePose();
    const sva::PTransformd X_actual = ctl.robot().frame("bracelet_link").position();
    Eigen::Vector3d lin_err = X_actual.translation() - X_compliance.translation();
    
    // Read the estimated/measured wrench being processed by the task
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

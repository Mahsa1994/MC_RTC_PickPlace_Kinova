#include "ImpedanceHoldState.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/ImpedanceTask.h>

void ImpedanceHoldState::configure(const mc_rtc::Configuration & config)
{
  config_ = config;
  if (config.has("stiffness")) task_stiffness_ = config("stiffness");
  if (config.has("damping"))   task_damping_   = config("damping");
  if (config.has("weight"))    task_weight_    = config("weight");
}

void ImpedanceHoldState::start(mc_control::fsm::Controller & ctl)
{
  dt_ = ctl.timeStep;

  // 1. Capture current EE pose as the hold target
  X_0_target_ = ctl.robot().frame("tool_frame").position();

  // 2. Create ImpedanceTask on tool_frame
  task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(
      ctl.robot().frame("tool_frame"),
      task_stiffness_,
      task_weight_);

  // 3. Cutoff period — 50ms is much more responsive than 300ms
  task_->cutoffPeriod(0.05);

  // 4. Kinematic tracking gains
  task_->stiffness(task_stiffness_);
  task_->damping(task_damping_);
  task_->targetPose(X_0_target_);

  // 5. Impedance model gains
  auto & gains = task_->gains();
  gains.mass().linear(Eigen::Vector3d(3.0, 3.0, 3.0));
  gains.mass().angular(Eigen::Vector3d(2.0, 2.0, 2.0));
  
  gains.spring().linear(Eigen::Vector3d(800.0, 800.0, 800.0));
  gains.spring().angular(Eigen::Vector3d(40.0, 40.0, 40.0));
  
  gains.damper().linear(Eigen::Vector3d(120.0, 120.0, 120.0));
  gains.damper().angular(Eigen::Vector3d(30.0, 30.0, 30.0));
  
  gains.wrench().linear(Eigen::Vector3d(1.0, 1.0, 1.0));
  gains.wrench().angular(Eigen::Vector3d(1.0, 1.0, 1.0));

  ctl.solver().addTask(task_);

  // 6. Sensor wiring diagnostic — uses the correct mc_rtc API
  bool sensor_found = false;
  for (const auto & fs : ctl.robot().forceSensors())
  {
    mc_rtc::log::info(
        "[ImpedanceHoldState] Robot has force sensor '{}' on body '{}'",
        fs.name(), fs.parentBody());
    if (fs.name() == "EEForceSensor")
      sensor_found = true;
  }
  if (!sensor_found)
  {
    mc_rtc::log::error(
        "[ImpedanceHoldState] 'EEForceSensor' not found in robot module! "
        "measuredWrench() will be zero — compliance will not work.");
  }
  else
  {
    // Confirm the task picked it up by reading back measuredWrench immediately
    // (will be zero at t=0 but confirms the path is wired)
    const auto w = task_->measuredWrench();
    mc_rtc::log::info(
        "[ImpedanceHoldState] EEForceSensor found. "
        "Initial measuredWrench: ({:.3f},{:.3f},{:.3f}) N",
        w.force().x(), w.force().y(), w.force().z());
  }
}

bool ImpedanceHoldState::run(mc_control::fsm::Controller & ctl)
{
  static int log_count = 0;
  if (++log_count % 200 == 0)
  {
    const sva::ForceVecd wrench_measured = task_->measuredWrench();
    const sva::PTransformd X_compliance  = task_->compliancePose();
    const sva::PTransformd X_actual      = ctl.robot().frame("tool_frame").position();

    Eigen::Vector3d deflection  = X_compliance.translation() - X_0_target_.translation();
    Eigen::Vector3d track_error = X_actual.translation() - X_compliance.translation();

    mc_rtc::log::info(
        "[ImpedanceHoldState] "
        "measuredWrench: ({:.2f},{:.2f},{:.2f}) N | "
        "Deflection: ({:.4f},{:.4f},{:.4f}) m (dist={:.4f}) | "
        "TrackErr: {:.4f} m",
        wrench_measured.force().x(),
        wrench_measured.force().y(),
        wrench_measured.force().z(),
        deflection.x(), deflection.y(), deflection.z(),
        deflection.norm(),
        track_error.norm());
  }

  return false;
} // ← this closing brace was missing, causing all downstream errors

void ImpedanceHoldState::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(task_);
  mc_rtc::log::info("[ImpedanceHoldState] Task removed");
}

EXPORT_SINGLE_STATE("KI::ImpedanceHoldState", ImpedanceHoldState)

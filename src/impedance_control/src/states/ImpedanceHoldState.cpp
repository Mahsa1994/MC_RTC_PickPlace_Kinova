#include "ImpedanceHoldState.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/ImpedanceTask.h>

// Load configuration
void ImpedanceHoldState::configure(const mc_rtc::Configuration &config)
{
  config_ = config;
  if (config.has("stiffness"))
    task_stiffness_ = config("stiffness");
  if (config.has("damping"))
    task_damping_ = config("damping");
  if (config.has("weight"))
    task_weight_ = config("weight");
  if (config.has("gains"))
    gains_config_ = config("gains");
  if (config.has("maxDeflection"))
    max_deflection_ = config("maxDeflection");
}

// State initialization: create impedance controller on EE frame
void ImpedanceHoldState::start(mc_control::fsm::Controller &ctl)
{
  dt_ = ctl.timeStep;

  // 1. Capture current EE pose as the hold target
  X_0_target_ = ctl.robot().frame("tool_frame").position();

  // 2. Create ImpedanceTask on tool_frame
  task_ = std::make_shared<mc_tasks::force::ImpedanceTask>(
      ctl.robot().frame("tool_frame"),
      task_stiffness_,
      task_weight_);

  // 3. Limits how fast wrench feedback affects control - 50ms is much more responsive than 300ms
  task_->cutoffPeriod(0.05);

  // 4. Kinematic tracking gains
  task_->stiffness(task_stiffness_);
  task_->damping(task_damping_);
  task_->targetPose(X_0_target_);

  // 5. Impedance model gains
  auto & gains = task_->gains();
  auto readV3 = [&](const char * grp, const char * key, const Eigen::Vector3d & def) -> Eigen::Vector3d {
  if (gains_config_.has(grp) && gains_config_(grp).has(key))
  {
      Eigen::Vector3d v = gains_config_(grp)(key);   // copy-init: uses Configuration's conversion operator
      return v;
  }
    return def;
  };
  gains.mass().linear(   readV3("mass",      "linear",  {3,3,3}));
  gains.mass().angular(  readV3("mass",      "angular", {2,2,2}));
  gains.spring().linear( readV3("stiffness", "linear",  {800,800,800}));
  gains.spring().angular(readV3("stiffness", "angular", {40,40,40}));
  gains.damper().linear( readV3("damping",   "linear",  {120,120,120}));
  gains.damper().angular(readV3("damping",   "angular", {30,30,30}));
  gains.wrench().linear( readV3("wrench",    "linear",  {1,1,1}));
  gains.wrench().angular(readV3("wrench",    "angular", {1,1,1}));

  ctl.solver().addTask(task_);

  // 6. Sensor wiring diagnostic - uses the correct mc_rtc API
  bool sensor_found = false;
  for (const auto &fs : ctl.robot().forceSensors())
  {
    mc_rtc::log::info(
        "[ImpedanceHoldState] Robot has force sensor '{}' on body '{}'",
        fs.name(), fs.parentBody());
    if (fs.name() == "EEForceSensor") // EEForceSensor is the virtual sensor giving wrench - not REAL external sensor
      sensor_found = true;
  }
  if (!sensor_found)
  {
    mc_rtc::log::error(
        "[ImpedanceHoldState] 'EEForceSensor' not found in robot module! "
        "measuredWrench() will be zero - compliance will not work.");
  }
  else
  {
    // Confirm the task picked it up by reading back measuredWrench immediately
    const auto w = task_->measuredWrench();
    mc_rtc::log::info(
        "[ImpedanceHoldState] EEForceSensor found. "
        "Initial measuredWrench: ({:.3f},{:.3f},{:.3f}) N",
        w.force().x(), w.force().y(), w.force().z());
  }
}

bool ImpedanceHoldState::run(mc_control::fsm::Controller &ctl)
{
  static int log_count = 0;
  if (++log_count % 200 == 0)
  {
    const sva::ForceVecd wrench_measured = task_->measuredWrench(); 
    const sva::PTransformd X_compliance = task_->compliancePose(); // Current compliance target pose (where impedance model thinks it should be)
    const sva::PTransformd X_actual = ctl.robot().frame("tool_frame").position(); // Actual end-effector pose from robot model

    Eigen::Vector3d deflection = X_compliance.translation() - X_0_target_.translation();
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

    // Sanity check: the bridge clamps its wrench estimate (see
    // kortex_mc_rtc_bridge_impedance.cpp), which bounds this in steady state,
    // but flag it loudly if it ever doesn't - this is not an automatic
    // safety stop, just a warning, since this single-state controller has
    // nowhere else to transition to yet.
    if (deflection.norm() > max_deflection_)
    {
      mc_rtc::log::warning(
          "[ImpedanceHoldState] Deflection {:.3f} m exceeds max_deflection_ ({:.3f} m) - "
          "check the bridge's wrench estimate/clamp.",
          deflection.norm(), max_deflection_);
    }
  }

  return false;
} 

void ImpedanceHoldState::teardown(mc_control::fsm::Controller &ctl)
{
  ctl.solver().removeTask(task_);
  mc_rtc::log::info("[ImpedanceHoldState] Task removed");
}

EXPORT_SINGLE_STATE("KI::ImpedanceHoldState", ImpedanceHoldState)

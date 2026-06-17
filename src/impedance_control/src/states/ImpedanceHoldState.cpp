#include "ImpedanceHoldState.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>

void ImpedanceHoldState::configure(const mc_rtc::Configuration & config)
{
  if (config.has("stiffness"))     task_stiffness_ = config("stiffness");
  if (config.has("damping"))       task_damping_   = config("damping");
  if (config.has("weight"))        task_weight_    = config("weight");
}

void ImpedanceHoldState::start(mc_control::fsm::Controller & ctl)
{
  dt_ = ctl.timeStep;

  // Capture current EE pose as the hold target (the "path point")
  X_0_target_ = ctl.robot().frame("bracelet_link").position();

  // TransformTask with LOW stiffness = compliant behavior:
  // - When pushed: arm yields because task force < external force
  // - When released: task pulls arm back to X_0_target_ (the path)
  // - Damping prevents oscillation on return
  //
  // Compliance law implicit in QP:  F_task = stiffness * x_err + damping * vel_err
  // The arm yields when F_ext > F_task, returns when F_ext = 0
  task_ = std::make_shared<mc_tasks::TransformTask>(
      ctl.robot().frame("bracelet_link"),
      task_stiffness_,
      task_weight_);

  // Set per-axis stiffness and damping explicitly
  // [angular stiffness(3), linear stiffness(3)]
  task_->stiffness(task_stiffness_);
  task_->damping(task_damping_);
  task_->target(X_0_target_);

  ctl.solver().addTask(task_);

  mc_rtc::log::success("[ImpedanceHoldState] Compliant hold active");
  mc_rtc::log::info("[ImpedanceHoldState] stiffness={} damping={} weight={}",
    task_stiffness_, task_damping_, task_weight_);
}

bool ImpedanceHoldState::run(mc_control::fsm::Controller & ctl)
{
  // The QP solver handles compliance implicitly — no wrench estimation needed.
  // task_->target() stays fixed at X_0_target_ (the path reference).
  // The arm naturally:
  //   1. Yields when pushed (external force > task restoring force)
  //   2. Returns to X_0_target_ when released (task dominates again)
  //   3. Damping prevents overshoot on return

  // Periodic logging of tracking error
  static int log_count = 0;
  if (++log_count % 200 == 0)
  {
    const sva::PTransformd X_actual = ctl.robot().frame("bracelet_link").position();
    Eigen::Vector3d lin_err = X_actual.translation() - X_0_target_.translation();
    mc_rtc::log::info("[ImpedanceHoldState] pos_err=({:.4f},{:.4f},{:.4f})m  "
                      "dist={:.4f}m",
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

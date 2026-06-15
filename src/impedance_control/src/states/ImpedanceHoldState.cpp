#include "ImpedanceHoldState.h"
#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>

// ── Configuration ─────────────────────────────────────────────────────────────
void ImpedanceHoldState::configure(const mc_rtc::Configuration & config)
{
  // Allow gains to be overridden from KinovaImpedance.yaml, e.g.:
  //   ImpedanceHold:
  //     base: KI::ImpedanceHoldState
  //     K: [10, 10, 10, 300, 300, 300]
  if (config.has("K")) { auto v = config("K"); K_ << v[0],v[1],v[2],v[3],v[4],v[5]; }
  if (config.has("D")) { auto v = config("D"); D_ << v[0],v[1],v[2],v[3],v[4],v[5]; }
  if (config.has("M")) { auto v = config("M"); M_ << v[0],v[1],v[2],v[3],v[4],v[5]; }
  if (config.has("task_stiffness")) task_stiffness_ = config("task_stiffness");
  if (config.has("task_weight"))    task_weight_    = config("task_weight");
}

// ── Start ─────────────────────────────────────────────────────────────────────
void ImpedanceHoldState::start(mc_control::fsm::Controller & ctl)
{
  dt_  = ctl.timeStep;
  vel_ = Eigen::Vector6d::Zero();

  // Capture current EE pose as the impedance reference
  X_0_target_ = ctl.robot().frame("bracelet_link").position();

  // TransformTask tracks a 6-DOF pose — no force sensor needed
  task_ = std::make_shared<mc_tasks::TransformTask>(
      ctl.robot().frame("bracelet_link"),
      task_stiffness_,
      task_weight_);
  task_->target(X_0_target_);
  ctl.solver().addTask(task_);

  mc_rtc::log::success("[ImpedanceHoldState] Holding at current EE pose");
  mc_rtc::log::info("[ImpedanceHoldState] K=[{},{},{},{},{},{}]  D=[{},{},{},{},{},{}]  M=[{},{},{},{},{},{}]",
    K_[0],K_[1],K_[2],K_[3],K_[4],K_[5],
    D_[0],D_[1],D_[2],D_[3],D_[4],D_[5],
    M_[0],M_[1],M_[2],M_[3],M_[4],M_[5]);
}

// ── Run (called every control cycle) ─────────────────────────────────────────
bool ImpedanceHoldState::run(mc_control::fsm::Controller & ctl)
{
  // ── 1. Read estimated wrench from the bridge
  // The bridge calls gc_->setWrenches({"EEForceSensor": wrench}) each cycle.
  // mc_rtc stores this in the robot's wrench map regardless of whether a
  // formal ForceSensor is registered — access via robot().data()->wrenches.
  // We use a try/catch so the state degrades to pure position hold if the
  // wrench isn't available yet (first few cycles).
  F_ext_ = sva::ForceVecd::Zero();
  try
  {
    // robot().forceSensor() throws if sensor not in module — use controller
    // wrenches map instead, which is populated by setWrenches() unconditionally
    const auto & wrenches = ctl.robot().forceSensors();
    for (const auto & fs : wrenches)
    {
      if (fs.name() == "EEForceSensor")
      {
        F_ext_ = fs.wrench();
        break;
      }
    }
  }
  catch (...) {}

  // ── 2. Impedance law: M·ẍ + D·ẋ + K·x_err = F_ext
  // x_err = deviation of the integrated target from the original hold pose
  // (we integrate onto X_0_target_ directly, so x_err is always 0 here —
  //  instead we treat the hold pose as a spring anchor and let the target drift)

  // Current actual EE pose
  const sva::PTransformd X_0_actual = ctl.robot().frame("bracelet_link").position();

  // Error: from anchor (original hold) to current integrated target
  // Angular: rotation error as a 3-vector
  Eigen::Vector3d ang_err = sva::rotationError(
      X_0_target_.rotation(), X_0_actual.rotation());
  Eigen::Vector3d lin_err = X_0_actual.translation() - X_0_target_.translation();

  Eigen::Vector6d x_err;
  x_err << ang_err, lin_err;

  // Impedance ODE integrated with Euler:
  //   acc = (F_ext - D*vel - K*x_err) / M
  //   vel += acc * dt
  Eigen::Vector6d acc = (F_ext_.vector() - D_.cwiseProduct(vel_) - K_.cwiseProduct(x_err))
                        .cwiseQuotient(M_);
  vel_ += acc * dt_;

  // ── 3. Integrate velocity onto the target pose
  Eigen::Vector3d dang = vel_.head<3>() * dt_;
  Eigen::Vector3d dlin = vel_.tail<3>() * dt_;

  // Build incremental transform: small rotation + translation
  sva::PTransformd delta(
      sva::RotX(dang.x()) * sva::RotY(dang.y()) * sva::RotZ(dang.z()),
      dlin);
  X_0_target_ = delta * X_0_target_;
  task_->target(X_0_target_);

  // ── 4. Periodic logging
  static int log_count = 0;
  if (++log_count % 500 == 0)
  {
    mc_rtc::log::info(
        "[ImpedanceHoldState] F_ext=({:.2f},{:.2f},{:.2f})N  "
        "vel=({:.4f},{:.4f},{:.4f})m/s  "
        "pos_err=({:.4f},{:.4f},{:.4f})m",
        F_ext_.force().x(), F_ext_.force().y(), F_ext_.force().z(),
        vel_[3], vel_[4], vel_[5],
        lin_err.x(), lin_err.y(), lin_err.z());
  }

  return false; // state never self-transitions
}

// ── Teardown ──────────────────────────────────────────────────────────────────
void ImpedanceHoldState::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.solver().removeTask(task_);
  mc_rtc::log::info("[ImpedanceHoldState] Task removed");
}

EXPORT_SINGLE_STATE("KI::ImpedanceHoldState", ImpedanceHoldState)

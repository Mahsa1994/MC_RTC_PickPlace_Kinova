#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/PostureTask.h>

struct HandGuideState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override
  {
    admTask_ = std::make_shared<mc_tasks::force::AdmittanceTask>(
        "tool_frame", ctl.robots(), ctl.robot().robotIndex());

    // admittance() takes sva::ForceVecd
    // TUNED 2026-09-02 (2nd pass): 0.0008/0.0010 (TUNING_GUIDE.md's
    // documented "free float" ceiling) produced clearer motion (~0.04 rad
    // swing under an ~8.7 Nm / 3.6 N sustained push) but still needed a
    // firm push to feel. No payload here (empty gripper, pure hand-
    // guiding), so pushing past the guide's documented range is reasonable
    // - tripling rather than creeping, to actually feel the difference
    // between iterations. Watch closely for oscillation/overshoot on
    // release this time (per guide step 5) - if it overshoots, back this
    // off or raise `damping` (currently 50.0) instead of lowering gain.
    admTask_->admittance(sva::ForceVecd(
        Eigen::Vector3d(0.0025, 0.0025, 0.0025),  // couple (torque)
        Eigen::Vector3d(0.0030, 0.0030, 0.0030))); // force

    // stiffness/damping take double (scalar)
    admTask_->stiffness(0.0001);
    // TUNED 2026-09-02 (3rd pass): admittance() tripled (see above) with
    // damping left at its original 50.0 still felt just as rigid - a 6x
    // gain increase producing no noticeable change means admittance isn't
    // the bottleneck anymore. damping is a genuine viscous brake on
    // achieved motion (TUNING_GUIDE.md), and was never touched while
    // admittance kept climbing, so it's the more likely limiter now.
    // Cutting it hard to isolate this before touching admittance again -
    // watch for oscillation on release (guide step 5); if it's unstable,
    // raise this back partway rather than lowering admittance.
    admTask_->damping(10.0);
    admTask_->weight(1000.0);

    // RAISED 2026-09-02: requested to help "still rigid" complaint, but the
    // achieved rate measured live (~0.003 rad/s) was only ~3% of the old
    // 0.10 rad/s cap - these were very unlikely to be the actual limiter.
    // Raising them anyway since it's harmless, but treat a stale build/
    // process (old binary still running) as the more likely explanation if
    // this doesn't change anything either.
    admTask_->maxLinearVel(Eigen::Vector3d(0.15, 0.15, 0.15));
    admTask_->maxAngularVel(Eigen::Vector3d(0.30, 0.30, 0.30));

    ctl.solver().addTask(admTask_);

    // BUILD-CHECK 2026-09-02: prints the task's ACTUAL live values right
    // after configuring it, so a fresh run's log unambiguously proves
    // whether this edited code is the code actually executing - compare
    // these numbers against whatever this file currently has hardcoded
    // above. If they don't match, the running binary is stale (old .so /
    // old process), not a physics problem.
    {
      const auto & adm = admTask_->admittance();
      mc_rtc::log::warning(
          "[HandGuideState] BUILD-CHECK: admittance couple=({:.4f},{:.4f},{:.4f}) "
          "force=({:.4f},{:.4f},{:.4f}) stiffness={:.4f} damping={:.4f} weight={:.1f} "
          "maxLinVel={:.3f} maxAngVel={:.3f}",
          adm.couple().x(), adm.couple().y(), adm.couple().z(),
          adm.force().x(), adm.force().y(), adm.force().z(),
          admTask_->stiffness(), admTask_->damping(), admTask_->weight(),
          admTask_->maxLinearVel().x(), admTask_->maxAngularVel().x());
    }

    postureTask_ = std::make_shared<mc_tasks::PostureTask>(
        ctl.solver(), ctl.robot().robotIndex(), 1.0, 1.0);
    ctl.solver().addTask(postureTask_);

    ctl.gui()->addElement({"Control"},
        mc_rtc::gui::Button("Stop Hand-Guiding", [this]() {
          stopRequested_ = true;
        }));

    mc_rtc::log::success("[HandGuideState] Active — push the arm!");
  }

  bool run(mc_control::fsm::Controller &) override
  {
    if(stopRequested_)
    {
      output("stopGuiding");
      return true;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    ctl.solver().removeTask(admTask_);
    ctl.solver().removeTask(postureTask_);
    ctl.gui()->removeCategory({"Control"});
    mc_rtc::log::info("[HandGuideState] Torn down.");
  }

private:
  std::shared_ptr<mc_tasks::force::AdmittanceTask> admTask_;
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  bool stopRequested_ = false;
};

EXPORT_SINGLE_STATE("KHG::HandGuideState", HandGuideState)

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
    // TUNED 2026-09-02: live test at 0.0005 (both axes) showed real motion
    // correlated with a sustained ~4 Nm push (joint_2/joint_3 both crept
    // several 0.001 rad over ~8-10s), confirming the whole pipeline
    // (wrench estimation, torque_sign, joint_trajectory_controller config)
    // works correctly - the gain itself was just far too conservative to
    // feel by hand. Raised to the top of TUNING_GUIDE.md's documented
    // range (0.001 "free float"), rotational kept slightly lower per the
    // guide's own step 4 ratio. Re-tune from here per TUNING_GUIDE.md
    // steps 3-5 if still too stiff or if it oscillates on release.
    admTask_->admittance(sva::ForceVecd(
        Eigen::Vector3d(0.0008, 0.0008, 0.0008),  // couple (torque)
        Eigen::Vector3d(0.0010, 0.0010, 0.0010))); // force

    // stiffness/damping take double (scalar)
    admTask_->stiffness(0.0001);
    admTask_->damping(50.0);
    admTask_->weight(1000.0);

    admTask_->maxLinearVel(Eigen::Vector3d(0.05, 0.05, 0.05));
    admTask_->maxAngularVel(Eigen::Vector3d(0.10, 0.10, 0.10));

    ctl.solver().addTask(admTask_);

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

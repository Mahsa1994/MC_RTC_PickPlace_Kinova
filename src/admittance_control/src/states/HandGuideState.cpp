#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/PostureTask.h>

struct HandGuideState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override
  {
    // Add admittance task
    auto admTask = std::make_shared<mc_tasks::force::AdmittanceTask>(
        "tool_frame", ctl.robots(), ctl.robot().robotIndex());

    admTask->admittance(sva::ForceVecd(
        Eigen::Vector6d::Constant(0.0005)));
    admTask->stiffness(sva::ForceVecd(
        Eigen::Vector6d::Constant(0.0001)));
    admTask->damping(sva::ForceVecd(
        Eigen::Vector6d::Constant(50.0)));
    admTask->weight(1000.0);

    Eigen::Vector3d maxLinVel(0.15, 0.15, 0.15);
    Eigen::Vector3d maxAngVel(0.30, 0.30, 0.30);
    admTask->maxLinearVel(maxLinVel);
    admTask->maxAngularVel(maxAngVel);

    ctl.solver().addTask(admTask);
    admTask_ = admTask;

    // Low-weight posture for nullspace
    postureTask_ = std::make_shared<mc_tasks::PostureTask>(
        ctl.solver(), ctl.robot().robotIndex(), 1.0, 1.0);
    ctl.solver().addTask(postureTask_);

    // GUI button to stop
    ctl.gui()->addElement({"Control"},
        mc_rtc::gui::Button("Stop Hand-Guiding", [this]() { stopRequested_ = true; }));

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

EXPORT_SINGLE_STATE("HandGuideState", HandGuideState)

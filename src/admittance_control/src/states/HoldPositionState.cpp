#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>

struct HoldPositionState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override
  {
    postureTask_ = std::make_shared<mc_tasks::PostureTask>(
        ctl.solver(), ctl.robot().robotIndex(), 10.0, 100.0);
    ctl.solver().addTask(postureTask_);

    ctl.gui()->addElement({"Control"},
        mc_rtc::gui::Button("Start Hand-Guiding", [this]() {
          startRequested_ = true;
        }));

    mc_rtc::log::success("[HoldPositionState] Holding position. Click to start hand-guiding.");
  }

  bool run(mc_control::fsm::Controller &) override
  {
    if(startRequested_)
    {
      output("startGuiding");
      return true;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    ctl.solver().removeTask(postureTask_);
    ctl.gui()->removeCategory({"Control"});
  }

private:
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  bool startRequested_ = false;
};

EXPORT_SINGLE_STATE("KHG::HoldPositionState", HoldPositionState)

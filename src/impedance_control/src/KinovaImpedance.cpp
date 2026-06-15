#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>

namespace mc_control
{

struct KinovaImpedance : public fsm::Controller
{
  KinovaImpedance(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
  : fsm::Controller(rm, dt, config)
  {
    mc_rtc::log::success("KinovaImpedance Controller loaded");
  }
};

} // namespace mc_control

CONTROLLER_CONSTRUCTOR("KinovaImpedance", mc_control::KinovaImpedance)

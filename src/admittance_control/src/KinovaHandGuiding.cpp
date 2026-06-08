#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_rtc/logging.h>

// In some versions of mc_rtc, this header is required for the macro. 
// If it still fails to find the header, comment this line out.
// #include <mc_control/mc_controller_loader.h> 

namespace mc_control
{

struct KinovaHandGuiding : public fsm::Controller
{
    KinovaHandGuiding(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
    : fsm::Controller(rm, dt, config)
    {
        mc_rtc::log::success("KinovaHandGuiding Controller loaded");
    }
};

} // namespace mc_control

// This macro exports the controller so mc_rtc can load it
CONTROLLER_CONSTRUCTOR("KinovaHandGuiding", mc_control::KinovaHandGuiding)

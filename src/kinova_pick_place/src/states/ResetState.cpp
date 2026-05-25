// src/states/ResetState.cpp
#include "kinova_pick_place/PickPlaceController.h"
#include <mc_control/fsm/State.h>

namespace kinova_pick_place {

struct ResetState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl_) override
  {
    auto & ctl = static_cast<PickPlaceController &>(ctl_);
    // Stay at current pose until a target arrives
    ctl.eeTask_->setTarget(ctl.robot("gen3").bodyPosW("tool_frame"));
    t_ = 0.0;
  }

  bool run(mc_control::fsm::Controller & ctl_) override
  {
    auto & ctl = static_cast<PickPlaceController &>(ctl_);
    rclcpp::spin_some(ctl.node_);
    t_ += ctl.dt_;

    if (ctl.pickPosition_.norm() > 0.01 && ctl.placePosition_.norm() > 0.01) {
      return true; // targets received -> transition to Approach
    }
    if (static_cast<int>(t_) % 5 == 0 && t_ - lastLog_ >= 1.0) {
      mc_rtc::log::info("ResetState: waiting for /pick_target and /place_target");
      lastLog_ = t_;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller &) override {}

private:
  double t_ = 0.0;
  double lastLog_ = 0.0;
};

} // namespace kinova_pick_place

EXPORT_SINGLE_STATE("Reset", ResetState)

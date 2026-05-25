// src/states/ApproachState.cpp
#include "kinova_pick_place/PickPlaceController.h"
#include <mc_control/fsm/State.h>
#include <sva/PTransformd.h>

namespace kinova_pick_place {

struct ApproachState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl_) override
  {
    auto & ctl = static_cast<PickPlaceController &>(ctl_);
    target_ = ctl.pickPosition_ + Eigen::Vector3d(0.0, 0.0, ctl.approachOffset_);
    ctl.eeTask_->setTarget(sva::PTransformd(ctl.pickOrientation_.toRotationMatrix(), target_));
    t_ = 0.0;
  }

  bool run(mc_control::fsm::Controller & ctl_) override
  {
    auto & ctl = static_cast<PickPlaceController &>(ctl_);
    t_ += ctl.dt_;
    if (ctl.nearTarget(target_, 0.02) || t_ > 5.0) return true;
    return false;
  }

private:
  Eigen::Vector3d target_;
  double t_ = 0.0;
};

} // namespace kinova_pick_place

EXPORT_SINGLE_STATE("Approach", ApproachState)

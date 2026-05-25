////
// src/states/PlaceState.cpp
#include "kinova_pick_place/PickPlaceController.h"
#include <mc_control/fsm/State.h>
#include <sva/PTransformd.h>

namespace kinova_pick_place {

struct PlaceState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl_) override
  {
    auto & ctl = static_cast<PickPlaceController &>(ctl_);
    ctl.eeTask_->setTarget(sva::PTransformd(ctl.placeOrientation_.toRotationMatrix(), ctl.placePosition_));
    t_ = 0.0;
    phase_ = 0; // 0: descend, 1: open gripper, 2: done
  }

  bool run(mc_control::fsm::Controller & ctl_) override
  {
    auto & ctl = static_cast<PickPlaceController &>(ctl_);
    t_ += ctl.dt_;

    if (phase_ == 0) {
      if (ctl.nearTarget(ctl.placePosition_, 0.015) || t_ > 4.0) {
        ctl.sendGripperCommand(0.8); // open
        phase_ = 1;
        t_ = 0.0;
      }
    } else if (phase_ == 1) {
      if (!ctl.gripperClosed_ || t_ > 2.0) {
        // clear targets so Reset waits for new ones
        ctl.pickPosition_.setZero();
        ctl.placePosition_.setZero();
        return true;
      }
    }
    return false;
  }

private:
  int phase_ = 0;
  double t_ = 0.0;
};

} // namespace kinova_pick_place

EXPORT_SINGLE_STATE("Place", PlaceState)


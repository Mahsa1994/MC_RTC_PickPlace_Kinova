#pragma once
#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_tasks/ImpedanceTask.h>

struct ImpedanceHoldState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
//  std::shared_ptr<mc_tasks::TransformTask> task_;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> task_;
  sva::PTransformd X_0_target_;  // fixed path reference — never changes
  mc_rtc::Configuration config_;
  mc_rtc::Configuration gains_config_;

  double dt_             = 0.005;
  double task_stiffness_ = 50.0;   // low = compliant (yields to pushes)
  double task_damping_   = 14.14;  // 2*sqrt(stiffness) for critical damping
  double task_weight_    = 100.0;  // low weight = yields in QP priority
};

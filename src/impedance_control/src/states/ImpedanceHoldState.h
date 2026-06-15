#pragma once
#include <mc_control/fsm/State.h>
#include <mc_tasks/TransformTask.h>
#include <SpaceVecAlg/SpaceVecAlg>

struct ImpedanceHoldState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  std::shared_ptr<mc_tasks::TransformTask> task_;

  // Hold target — updated each cycle by impedance integration
  sva::PTransformd X_0_target_;

  // Virtual velocity state [angular(3); linear(3)]
  Eigen::Vector6d vel_ = Eigen::Vector6d::Zero();

  double dt_ = 0.005;

  // Impedance gains [angular(3); linear(3)] — overridable from YAML
  Eigen::Vector6d K_{ (Eigen::Vector6d() << 10, 10, 10, 200, 200, 200).finished() };
  Eigen::Vector6d D_{ (Eigen::Vector6d() << 6.32, 6.32, 6.32, 28.28, 28.28, 28.28).finished() };
  Eigen::Vector6d M_{ (Eigen::Vector6d() << 1, 1, 1, 1, 1, 1).finished() };

  // TransformTask gains
  double task_stiffness_ = 200.0;
  double task_weight_    = 1000.0;

  // Estimated wrench injected by the bridge (read from controller's wrench store)
  sva::ForceVecd F_ext_ = sva::ForceVecd::Zero();
};

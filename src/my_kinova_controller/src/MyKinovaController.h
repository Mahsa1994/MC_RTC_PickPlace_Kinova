#pragma once
#include <mc_control/mc_controller.h>
#include <mc_solver/CollisionsConstraint.h>

struct MyKinovaController : public mc_control::MCController
{
  MyKinovaController(mc_rbdyn::RobotModulePtr rm,
                     double dt,
                     const mc_rtc::Configuration & config);
  bool run() override;
  void reset(const mc_control::ControllerResetData & data) override;

private:
  mc_solver::CollisionsConstraint collisionsConstraint;
};

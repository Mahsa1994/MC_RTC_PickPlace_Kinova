#include "MyKinovaController.h"

MyKinovaController::MyKinovaController(mc_rbdyn::RobotModulePtr rm,
                                       double dt,
                                       const mc_rtc::Configuration & config)
: MCController(rm, dt, config),
  collisionsConstraint(robots(), 0, 0, dt)
{
//  solver().addConstraintSet(contactConstraint);
//  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);

  // Only pairs confirmed safe from retract position
  // Based on GUI distances: shoulder::forearm=31.86cm, base::forearm=46.82cm
  // forearm::bracelet=37.47cm, base::swrist1=16.78cm, base::swrist2=16.70cm
  collisionsConstraint.addCollisions(solver(), {
    {"shoulder_link", "forearm_link",           0.15, 0.05, 0.0},
    {"base_link",     "forearm_link",           0.15, 0.05, 0.0},
    {"forearm_link",  "bracelet_link",          0.15, 0.05, 0.0},
    {"base_link",     "spherical_wrist_1_link", 0.08, 0.02, 0.0},
    {"base_link",     "spherical_wrist_2_link", 0.08, 0.02, 0.0},
  });
//  solver().addConstraintSet(collisionsConstraint);

  mc_rtc::log::success("MyKinovaController init done");
}

void MyKinovaController::reset(const mc_control::ControllerResetData & data)
{
  MCController::reset(data);
}

bool MyKinovaController::run()
{
  return MCController::run();
}

CONTROLLER_CONSTRUCTOR("MyKinovaController", MyKinovaController)

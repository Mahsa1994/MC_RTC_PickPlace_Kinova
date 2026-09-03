#pragma once

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/logging.h>
#include <cmath>

// Kinova's own Gen3 documentation lists two wrist singularities. Both are
// checked here because neither is fixable by tuning: at a singularity the arm
// has fewer Cartesian DOF than the 6-D admittance task is asking for, so the
// missing direction simply cannot be produced.
//
// "Joints 4 and 6 singularity: Joint 5 is at 0 deg so joints 4 and 6 are
//  perfectly aligned and have the same effect."
// Measured against the URDF Jacobian at the logged guiding pose:
//   joint_5 =  0.000  sigma_min = 0.0000  axes 4/6 at   0.0 deg  EXACTLY singular
//   joint_5 = -0.181  sigma_min = 0.0325  axes 4/6 at  10.4 deg  (2026-09-03 run)
//   joint_5 = -1.200  sigma_min = 0.1273  axes 4/6 at  68.8 deg
//   joint_5 = +1.200  sigma_min = 0.1565  axes 4/6 at  68.8 deg  <-- best
// The chain is not symmetric about 0, so +1.2 is preferred over -1.2 even
// though the axis angle is identical: sigma_min is 23% higher.
//
// Beyond killing rotational guiding, proximity to this singularity is what
// drives the bridge's model-vs-real safety gate. With axes 4 and 6 nearly
// collinear, joint_4 and joint_6 counter-rotating produce almost no Cartesian
// motion, so the QP can wander freely along that null space. Re-anchoring the
// admittance TARGET each tick does not correct it - the drift is invisible in
// Cartesian space by construction - which is why every gate trip in the logs
// names joint_4 and no other joint.
namespace khg
{

constexpr double kWristSingularBand = 0.35; // rad, ~20 deg either side of 0
constexpr double kAlignSingularBand = 0.20; // rad, "Joints 1 and 6 singularity"
constexpr double kWristParkTarget   = 1.20; // rad, best-conditioned joint_5

inline double jointQ(const mc_rbdyn::Robot & r, const char * name)
{
  return r.mbc().q[r.jointIndexByName(name)][0];
}

/// True when joint_5 is close enough to 0 that rotational guiding cannot work.
inline bool wristSingular(const mc_rbdyn::Robot & r)
{
  return std::abs(jointQ(r, "joint_5")) < kWristSingularBand;
}

/// Logs loudly rather than moving the arm on its own: silently rotating the
/// wrist out of a singularity while an operator has their hands on it is a
/// worse failure than refusing to work and saying why. Parking is an explicit
/// operator action in HoldPosition, never automatic and never while guiding.
inline bool warnIfSingular(const mc_rbdyn::Robot & r, const char * tag, const char * when)
{
  bool singular = false;
  const double q5 = jointQ(r, "joint_5");
  if(std::abs(q5) < kWristSingularBand)
  {
    singular = true;
    mc_rtc::log::error(
        "[{}] {}: JOINTS 4/6 WRIST SINGULARITY - joint_5 = {:.3f} rad ({:.1f} deg). "
        "Axes 4 and 6 are aligned, so the wrist has 2 of 3 rotation DOF and rotational "
        "guiding CANNOT work here at any gain. Use \"Park Wrist\" in HoldPosition to "
        "drive joint_5 to {:+.2f} rad.",
        tag, when, q5, q5 * 180.0 / M_PI, kWristParkTarget);
  }
  const double align = jointQ(r, "joint_2") + jointQ(r, "joint_3") + q5;
  if(std::abs(align) < kAlignSingularBand)
  {
    singular = true;
    mc_rtc::log::error(
        "[{}] {}: JOINTS 1/6 SINGULARITY - theta_2+theta_3+theta_5 = {:.3f} rad. "
        "Axes 1 and 6 are aligned; yaw guiding will be degraded.",
        tag, when, align);
  }
  return singular;
}

} // namespace khg

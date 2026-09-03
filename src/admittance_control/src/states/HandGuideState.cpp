#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/PostureTask.h>
#include <cmath>

#include "WristSingularity.h"


struct HandGuideState : mc_control::fsm::State
{
  void start(mc_control::fsm::Controller & ctl) override
  {
    admTask_ = std::make_shared<mc_tasks::force::AdmittanceTask>(
        "tool_frame", ctl.robots(), ctl.robot().robotIndex());

    // admittance() takes sva::ForceVecd
    // TUNED 2026-09-02 (2nd pass): 0.0008/0.0010 (TUNING_GUIDE.md's
    // documented "free float" ceiling) produced clearer motion (~0.04 rad
    // swing under an ~8.7 Nm / 3.6 N sustained push) but still needed a
    // firm push to feel. No payload here (empty gripper, pure hand-
    // guiding), so pushing past the guide's documented range is reasonable
    // - tripling rather than creeping, to actually feel the difference
    // between iterations. Watch closely for oscillation/overshoot on
    // release this time (per guide step 5) - if it overshoots, back this
    // off or raise `damping` (currently 50.0) instead of lowering gain.
    // TUNED 2026-09-03 (couple only), then CORRECTED DOWN the same day.
    // First pass raised couple 0.0025 -> 0.05 on the premise that the measured
    // |M| (median 1.64 Nm, p95 5.79 Nm) was operator-intended twist and the
    // wrist was simply under-driven. Decomposing the moment against the
    // simultaneous force refuted that premise:
    //   93% of |M| is PERPENDICULAR to F  (lever arm, not twist)
    //   implied moment arm |M|/|F|: median 18 cm, p75 51 cm
    //   true parallel twist: median |M_par| only 1.13 Nm
    // i.e. most of the moment seen at tool_frame is the operator pushing the
    // arm 20-50 cm away from tool_frame (wrist/forearm), not asking for
    // rotation. The task is referenced at tool_frame and cannot distinguish
    // the two, so at 0.05 every off-point linear push spun the wrist, while
    // deliberate rotation - which can only muster ~1 Nm of genuine couple -
    // still barely moved. Amplifying that channel 20x amplified the artifact.
    // 0.01 is 4x the original: enough that a real couple at the gripper is
    // feelable, low enough that an 18 cm-lever push yields ~0.03 rad/s of
    // parasitic rotation instead of ~0.15.
    // Note a single fixed contact offset fits the data poorly (least-squares
    // |d| = 55 cm, R^2 = 0.157), so there is no consistent grab point to
    // compensate for here - the operator's hand moved during the run. Making
    // off-tool pushes translate cleanly needs the contact point estimated and
    // the admittance frame moved to it (or a joint-space admittance), which is
    // a design change, not a gain. Until then: push on the gripper body.
    // SUPERSEDED 2026-09-03 (same day): both of the above passes were chasing a
    // gain for a problem that was not a gain problem. The arm was being guided
    // 6.9 deg from the documented joints-4/6 wrist singularity (see
    // WristSingularity.h), where one rotational DOF does not
    // exist. 0.0025 felt dead, 0.05 spun parasitically on lever-arm moments,
    // and NEITHER could have produced real rotation. 0.03 is a middle value to
    // re-evaluate from a non-singular pose; it is a starting point, not a
    // tuned result. Retune only after confirming |joint_5| > kWristSingularBand.
    admTask_->admittance(sva::ForceVecd(
        Eigen::Vector3d(0.03, 0.03, 0.03),        // couple (torque)
        Eigen::Vector3d(0.0030, 0.0030, 0.0030))); // force

    // stiffness/damping take double (scalar)
    // TUNED 2026-09-02 (4th pass): read AdmittanceTask's actual source
    // (update() in AdmittanceTask.cpp) after admittance/damping/maxVel
    // changes all failed to change the felt response. Confirmed the
    // velocity law itself (admittance * wrenchError) has nothing else
    // attenuating it - but stiffness()/weight() are inherited unchanged
    // from the underlying TransformTask and govern something completely
    // different: how hard the QP actually drives the real joints toward
    // the (separately, already force-computed) moving target - NOT a
    // spring back to a fixed start pose, since the target itself already
    // moves via the admittance law above. At 0.0001 (TUNING_GUIDE.md's
    // literal "near-zero" advice) this tracking gain may simply be too
    // weak to produce enough torque to move real joint friction/stiction,
    // regardless of how fast the internal target races ahead - this is the
    // one major parameter left untouched through 3 prior tuning passes.
    // Raised to the guide's own documented ceiling ("never set > 1") to
    // test this directly, isolated from every other change.
    admTask_->stiffness(1.0);
    // TUNED 2026-09-02 (3rd pass): admittance() tripled (see above) with
    // damping left at its original 50.0 still felt just as rigid - a 6x
    // gain increase producing no noticeable change means admittance isn't
    // the bottleneck anymore. damping is a genuine viscous brake on
    // achieved motion (TUNING_GUIDE.md), and was never touched while
    // admittance kept climbing, so it's the more likely limiter now.
    // Cutting it hard to isolate this before touching admittance again -
    // watch for oscillation on release (guide step 5); if it's unstable,
    // raise this back partway rather than lowering admittance.
    admTask_->damping(10.0);
    admTask_->weight(1000.0);

    // RAISED 2026-09-02: requested to help "still rigid" complaint, but the
    // achieved rate measured live (~0.003 rad/s) was only ~3% of the old
    // 0.10 rad/s cap - these were very unlikely to be the actual limiter.
    // Raising them anyway since it's harmless, but treat a stale build/
    // process (old binary still running) as the more likely explanation if
    // this doesn't change anything either.
    admTask_->maxLinearVel(Eigen::Vector3d(0.15, 0.15, 0.15));
    admTask_->maxAngularVel(Eigen::Vector3d(0.30, 0.30, 0.30));

    ctl.solver().addTask(admTask_);

    // BUILD-CHECK 2026-09-02: prints the task's ACTUAL live values right
    // after configuring it, so a fresh run's log unambiguously proves
    // whether this edited code is the code actually executing - compare
    // these numbers against whatever this file currently has hardcoded
    // above. If they don't match, the running binary is stale (old .so /
    // old process), not a physics problem.
    {
      const auto & adm = admTask_->admittance();
      mc_rtc::log::warning(
          "[HandGuideState] BUILD-CHECK: admittance couple=({:.4f},{:.4f},{:.4f}) "
          "force=({:.4f},{:.4f},{:.4f}) stiffness={:.4f} damping={:.4f} weight={:.1f} "
          "maxLinVel={:.3f} maxAngVel={:.3f}",
          adm.couple().x(), adm.couple().y(), adm.couple().z(),
          adm.force().x(), adm.force().y(), adm.force().z(),
          admTask_->stiffness(), admTask_->damping(), admTask_->weight(),
          admTask_->maxLinearVel().x(), admTask_->maxAngularVel().x());
    }

    postureTask_ = std::make_shared<mc_tasks::PostureTask>(
        ctl.solver(), ctl.robot().robotIndex(), 1.0, 1.0);
    ctl.solver().addTask(postureTask_);

    ctl.gui()->addElement({"Control"},
        mc_rtc::gui::Button("Stop Hand-Guiding", [this]() {
          stopRequested_ = true;
        }));

    khg::warnIfSingular(ctl.realRobot(), "HandGuideState", "at entry");
    mc_rtc::log::success("[HandGuideState] Active — push the arm!");
  }

  bool run(mc_control::fsm::Controller & ctl) override
  {
    // ANTI-WINDUP 2026-09-03. AdmittanceTask::update() integrates its target
    // open-loop (`target(delta * target())`) - it never re-references where
    // the arm actually is. Behind the bridge's delta_max clamp the real arm
    // always follows with a small per-tick deficit, and because the target
    // integrates from itself that deficit accumulates without bound: the
    // 2026-09-03 log shows model-vs-real creeping to 0.0495 rad over ~12 s of
    // guiding, then tripping model_real_gate (0.05) and latching.
    // Re-anchoring the target to the measured pose each tick turns the task
    // from a position integrator into pure velocity following: the position
    // error is then always exactly one timestep of refVel, so it cannot wind
    // up, and the QP is driven by the feedforward velocity as intended.
    admTask_->targetPose(ctl.realRobot().frame("tool_frame").position());

    // Re-check every 2 s: the operator can guide the arm INTO a singularity
    // mid-session, at which point rotation dies with no other visible symptom.
    if(++tick_ % 2000 == 0) { khg::warnIfSingular(ctl.realRobot(), "HandGuideState", "during guiding"); }

    if(stopRequested_)
    {
      output("stopGuiding");
      return true;
    }
    return false;
  }

  void teardown(mc_control::fsm::Controller & ctl) override
  {
    ctl.solver().removeTask(admTask_);
    ctl.solver().removeTask(postureTask_);
    ctl.gui()->removeCategory({"Control"});
    mc_rtc::log::info("[HandGuideState] Torn down.");
  }

private:
  std::shared_ptr<mc_tasks::force::AdmittanceTask> admTask_;
  std::shared_ptr<mc_tasks::PostureTask> postureTask_;
  bool stopRequested_ = false;
  unsigned long tick_ = 0;
};

EXPORT_SINGLE_STATE("KHG::HandGuideState", HandGuideState)

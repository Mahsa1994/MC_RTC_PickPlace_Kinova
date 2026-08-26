# MC_RTC_PickPlace_Kinova

Master's project (FRQNT-funded, see `FRQNT_Proposal.pdf`): an adaptive
control framework for a Kinova Gen3 that adjusts its motion based on
inferred human emotional state/comfort during a collaborative pick-and-place
task, to preserve emotional (not just physical) safety in HRC. This file is
the running reference for what's been built, what's known to be broken or
fixed, and the open decisions - update it as the project moves, don't let it
go stale.

## Repo layout

- `src/impedance_control` - the compliance/impedance controller (mc_rtc FSM,
  currently one state: hold a fixed pose under compliance). Real hardware
  and Gazebo both drive it through the same bridge,
  `kortex_mc_rtc_bridge_impedance` (estimates external wrench from joint
  torques + Jacobian transpose - no physical F/T sensor).
- `src/pick_and_place` - the actual pick-and-place FSM (`PickPlaceController`),
  home/pick/place poses, gripper action client. As of the `pick_and_place`
  branch, its transport legs reuse the impedance_control bridge and a new
  compliant trajectory state (see below).
- `src/gripper_control`, `src/admittance_control`, `src/my_kinova_controller`,
  `src/kinova_pick_place` - other experiments in this repo; not touched this
  session.
- `research/multiphysio_poc` - teacher/student distillation proof of concept
  against the public MultiPhysio-HRC dataset (see its own README).

## Branches

- `real_robot_impedance` - the impedance_control bug fixes below.
- `pick_and_place` (branched from `real_robot_impedance`) - the fixes plus
  the compliant pick-and-place extension. **Not yet committed** - both sets
  of changes are sitting uncommitted; commit them as two separate commits
  (fixes, then the pick-and-place feature) when ready, don't squash them
  together.

## impedance_control: bugs found and fixed (2026-07-27)

Symptom reported: arm sometimes moves jerkily with nobody touching it, and
in some configurations "goes crazy" when it does get touched. Root causes
and fixes, all in `kortex_mc_rtc_bridge_impedance.cpp` unless noted:

1. **Jerky with no contact** - the inverse-dynamics bias computation forced
   joint acceleration (`alphaD`) to zero, so any time the arm was actually
   accelerating (which is almost always, even just correcting jitter while
   "holding still"), the real inertial torque `M(q)*qddot` got misread as
   external contact force. Fixed by estimating `qddot` via a filtered
   finite-difference of encoder velocity and feeding it into the bias
   computation instead of zero (`accel_filter_tau` param, default 0.1s).
2. **"Goes crazy" at specific configurations** - the external wrench was
   solved via a plain Jacobian-transpose inverse
   (`completeOrthogonalDecomposition()`), which blows up near kinematic
   singularities (routine during reach/pick-place motion): torque-sensor
   noise gets amplified into huge spurious force spikes. Fixed with damped
   least squares (`wrench_dls_lambda2` param, default 4.0) plus a hard clamp
   on the resulting force/moment magnitude (`max_force_estimate` /
   `max_moment_estimate`, default 60N / 15Nm) before it ever reaches the
   impedance task.
3. **Compliance effectively disabled during real motion** - the wrench was
   gated to zero above 0.15 rad/s (~8.6°/s), well below normal pick-and-place
   speed. Raised to a parameterized `qd_gate_low`/`qd_gate_high` (default
   0.05/0.3 rad/s) now that (1) is fixed - **re-validate against real
   push-while-moving tests**, these defaults are reasoned guesses, not
   measured.
4. **Safety-gate contradiction** - `impedance_real.launch.py` had
   `dry_run: False` with a comment saying "flip to False only after sign
   check," i.e. it had already violated its own stated safety gate (the
   prior commit was literally "set default DRY_RUN to false"). Reverted to
   `dry_run: True`. **Do not flip back to False until you've done the
   verification procedure below** - all of (1)-(3) above are unvalidated on
   real hardware.
5. Lower severity: removed a dead `wrenchThreshold` YAML key that was never
   read by `ImpedanceHoldState` (real deadbanding happens in the bridge, via
   `deadband_force`/`deadband_moment`); removed dead/commented-out code;
   added a (warning-only, not yet a safety stop) deflection sanity check;
   aligned `Timestep` across `KinovaImpedance.yaml`/`mc_rtc.yaml`/the
   bridge's `loop_dt` (was 0.001 vs 0.002 vs 0.001 - a mismatch here desyncs
   the impedance model's integration rate from wall-clock time, which can
   turn an intentionally overdamped virtual spring-damper into an
   underdamped one in real time); fixed the README's stale sensor-name
   reference (`EEForceSensor`, matching the code, not
   `EndEffectorForceSensor`).

### Before running on real hardware again

1. ~~Build and read the code changes~~ **Done 2026-08-19** - built and
   extensively tested on real hardware, see the pick_and_place validation
   session below.
2. ~~Launch with `dry_run: true`, manually push in every direction, confirm
   sign~~ **Done for translation** (2 directions, `HoldCurrent`), see below.
   Rotational/moment sign was never checked - still open.
3. ~~Only then set `dry_run: false`~~ **Done for `HoldCurrent`** with an
   E-stop operator present - confirmed real yield + return-to-position on
   real hardware. The full pick-and-place loop (`MoveHome`/`MoveToPick`/
   gripper/`MoveToPlace`/`ReturnHome`) has **not** been live-validated yet -
   see "Current state" below before flipping `dry_run: false` for the full
   loop.
4. **Still open**: re-validate `wrench_dls_lambda2`, `max_force_estimate`,
   `max_moment_estimate`, `qd_gate_low`, `qd_gate_high` against real,
   *moving* pick-and-place trajectories (not just static holds) - these are
   still the reasoned-guess defaults from 2026-07-27, never re-tuned against
   real motion data.

## pick_and_place branch: compliant pick-and-place extension

`PickPlaceController` already had `CartesianMove` (rigid, BSplineTrajectoryTask)
with a configurable target/waypoints/duration, `JointMove`, a real (not stub)
`Gripper` action client, and `pick_pose`/`place_pose`/`home_pose` config - but
no compliance at all (its own bridge, `kortex_mc_rtc_bridge.cpp`, never reads
joint effort, so it has no wrench to react to).

Added:

- **`ComplianceCartesianMove`** (`src/pick_and_place/src/states/PickPlaceStates.cpp`) -
  same target/waypoints/duration config surface as `CartesianMove`, but
  tracks the path with an `ImpedanceTask` instead of a stiff trajectory task.
  If the measured wrench exceeds `contact_force_threshold` (default 8N -
  must stay above whatever `deadband_force` the bridge is launched with),
  the trajectory clock pauses (the moving target stops advancing) until the
  wrench clears for `clear_hold_time` (default 0.3s), then resumes toward
  the same waypoints/target - "yields, then gets back to it," as requested,
  for a moving target rather than just a held pose.
- Wired the full FSM sequence in `PickPlaceController.yaml`:
  `Unfold -> MoveHome -> MoveToPick(compliant) -> CloseGripper -> MoveToPlace(compliant) -> OpenGripper -> ReturnHome -> Idle`.
  **`MoveToPlace` is the experiment-manipulation state**: its `waypoints`
  set the trajectory shape and `duration` sets the average velocity - both
  changeable per trial without recompiling, per the proposal's requirement
  to vary trajectory and velocity across trials.
- Both `pick_place_real.launch.py` and `pick_place_gazebo.launch.py` now
  launch `impedance_control`'s `kortex_mc_rtc_bridge_impedance` (wrench-aware)
  instead of `pick_and_place`'s own bare bridge, so `ComplianceCartesianMove`
  has a real wrench to read. Added `impedance_control` as an `exec_depend` in
  `pick_and_place/package.xml`. The old bridge is left in place (unused by
  the launch files now) with a note pointing at the replacement - not
  deleted, in case you still want a simple rigid fallback.
- `home_pose`/`pick_pose`/`place_pose` and the `Unfold` joint-angle
  calibration were **not touched** - those are physical calibration values
  that couldn't be verified from where this was written.

~~Same "not compiled, not tested on hardware" caveat as above applies here
too.~~ **Superseded 2026-08-19** - see the validation session immediately
below.

## pick_and_place: real-hardware validation session (2026-08-19)

Full-day, iterative, real-hardware debugging and validation session on the
`pick_and_place` branch, always via `pick_place_real.launch.py`. Documenting
in the order problems were found, since several of these masked or were
mistaken for each other before the actual cause was isolated.

### Problems found and fixed

1. **Duplicate Kortex driver launch → RT overruns + `WRONG_SERVOING_MODE`
   faults.** `pick_place_real.launch.py` already launches the Kortex driver
   itself (`kortex_bringup gen3.launch.py`) - manually launching it again
   beforehand starts two competing driver instances, which collided
   (`controller_manager: already loaded a urdf`, `can not be configured from
   'active' state'`) and cascaded into `WRONG_SERVOING_MODE`. Separately
   (visible once deduplicated), `ros2_control_node` ran under default
   `SCHED_OTHER` with no elevated priority, causing intermittent 1kHz loop
   overruns since the Kortex API's low-level write call alone takes ~1ms,
   leaving no margin. Fixed: don't double-launch; added `chrt -f 80` (SCHED_
   FIFO) as a prefix to the `kortex` `ExecuteProcess` in
   `pick_place_real.launch.py` so `ros2_control_node` inherits real-time
   priority on fork/exec.
2. **Wrong controller silently loaded.** `~/.config/mc_rtc/mc_rtc.yaml` had
   been copied from `impedance_control/etc/mc_rtc.yaml`, which sets `Enabled:
   KinovaImpedance` and only lists `impedance_control`'s module/states paths.
   `pick_and_place`'s `PickPlaceController` was never actually being loaded
   or exercised by any prior testing that used this file. Fixed: created
   `pick_and_place/etc/mc_rtc.yaml` with `Enabled: PickPlaceController`, the
   correct module/states paths, and `Timestep: 0.005` (pick_and_place's
   intended 200Hz, not impedance_control's 1kHz - mismatched Timestep here
   was a second, separate desync risk of the same kind documented in the
   2026-07-27 section above).
3. **`ObserverPipeline` config crash.** The new `mc_rtc.yaml`'s
   `ObserverPipelines.observers` block was initially copied in a malformed
   form (not a list, missing `type:`) - crashed `PickPlaceController` on
   startup with an uncaught `fmt::format_error` inside
   `mc_observers::ObserverPipeline::create`. Fixed by matching the
   list-with-`type: Encoder` form already used correctly elsewhere in the
   repo (`KinovaImpedance.yaml`).
4. **`joint_trajectory_controller` silently rejecting every command.** Once
   `dry_run:false` testing began, the arm stayed completely rigid under a
   firm push - `ros2_control_node` was logging a continuous
   `Velocity of last trajectory point ... is not zero` error. Root cause:
   `joint_trajectory_controller` rejects any trajectory whose final point has
   nonzero velocity unless `allow_nonzero_velocity_at_trajectory_end: true`;
   the bridge streams short/near-single-point trajectories every control
   cycle, so any actively-compliant (i.e. moving) joint almost always has
   nonzero velocity at that point. Fixed by adding the parameter to both
   `joint_trajectory_controller` and `kinova_joint_controller` in
   `pick_and_place/config/ros2_controllers.yaml`. Confirmed fixed: retested
   after, real motion + correct yield + return-to-position observed live.
5. **Sign-check methodology gave a confusing, precise-but-wrong result -
   turned out to be the diagnostic, not the controller.** Comparing
   `ImpedanceTask::measuredWrench()` against `compliancePose()`/
   `deltaCompliancePose()` axis-by-axis showed a suspiciously exact but
   *inverted* relationship (`deflection = -force / spring_stiffness`).
   Chased this through three plausible fixes - flipping `torque_sign`,
   negating the wrench in the bridge, negating the `wrench` impedance gain -
   none of which could have worked (all three only rescale the same upstream
   signal that both the log and the compliance law derive from, so they
   preserve its relative sign no matter what) and the last of which turned
   out to silently zero the compliance response entirely (negative gains
   appear to get clamped internally). Actual cause: `measuredWrench()` is in
   the tool/surface frame, `compliancePose()` is in the world frame, and the
   arm's pose has a non-trivial rotation between them - comparing them
   axis-by-axis without accounting for that rotation is invalid. Fixed by
   comparing `ctl.robot().frame(ee_frame_).position()` (always world-frame,
   unambiguous) against the fixed anchor instead, and judging it against the
   physically-known push direction rather than the wrench reading. Passed
   cleanly on the first attempt with the corrected diagnostic.
6. **`Unfold`'s hardcoded joint target didn't match the arm's actual resting
   pose.** Off by up to ~140 deg on 3 of 6 joints - correctly rejected by the
   bridge's own `FIRST CMD MISMATCH` safety gate (large first-command jumps
   are refused, not published), which meant `Unfold` could never actually
   run. The mismatch was identical and reproducible across every real-
   hardware log this session, strongly suggesting it's genuinely where the
   arm rests between runs, not a one-off. Fixed by updating the target to
   match. **Still needs your confirmation**: is that resting pose actually
   the intended experiment starting pose, or just wherever the arm happened
   to be left? Update the target again if not.
7. **Default compliance gains too stiff for hand-guided use.**
   `ImpedanceTask` defaults (`stiffness: 800` N/m linear, `40` Nm/rad
   angular) meant a 1N push only earned ~1.25mm of give. Softened
   iteratively with live feedback; also found by direct comparison that an
   intermediate guess (`stiffness: 200`) was still 2x stiffer than
   `impedance_control/etc/KinovaImpedance.yaml`'s already-validated linear
   value (`100`). Landed on, applied consistently to `HoldCurrent`,
   `MoveToPick`, and `MoveToPlace`:
   `mass: [3,3,3]/[2,2,2]` (linear/angular),
   `stiffness: [100,100,100]/[10,10,10]`,
   `damping: [60,60,60]/[15,15,15]`.
8. **`home_pose` was wrong by ~33cm in translation** (and, it later turned
   out, drastically wrong in rotation too). First live `MoveHome` test: arm
   moved unexpectedly fast/loud - E-stopped. First fix attempt: direct
   comparison against the Kinova web app's own Monitoring snapshot
   (`Monitoring_2026-08-19_4-10-15.json`) taken with the arm sitting at its
   real Home - translation changed `[0.287, 0.175, 0.392]` ->
   `[0.572, 0.016, 0.422]`, assumed high-confidence since units/frame
   *seemed* like they should correspond 1:1. **This assumption was wrong**
   (see problem 11's update, 2026-08-24 below) - the web app's reported EE
   translation does not actually match mc_rtc's own `tool_frame` FK for the
   same physical pose, evidently a different reference point along the tool
   (TCP vs flange or similar). Rotation was left unverified at the time -
   `poseFromConfig()`'s `[roll,pitch,yaw]` convention doesn't obviously
   correspond to Kinova's web-app thetaX/thetaY/thetaZ, and the real Home
   orientation sits near gimbal lock in this convention (pitch ≈ -90°),
   which makes a Euler-angle diagnostic unreliable exactly where it's
   needed. A rotation-verification log was added to `CartesianMove::start()`
   for this, but its output was, at the time, judged inconclusive - the
   *next* two incidents turned out to be execution-pacing/model-divergence
   (below), not this rotation value, so it stayed formally unverified until
   problem 11's update resolved it directly.
9. **Two more live E-stops testing `MoveHome`, after the translation fix -
   root cause was execution pacing, not pose.** With the arm confirmed
   sitting at real Home: (a) it started moving in an unexpected direction
   ("not the default direction towards Home"); after re-testing with the
   arm still at Home, (b) it moved straight up and then continuously away
   from Home (arm never stopped drifting) even after `delta_max` had
   already been cut 20x for a slower retest. Traced to
   `kortex_mc_rtc_bridge_impedance.cpp`'s publish block: `delta_max`
   clamped the per-cycle step around the *current* encoder position but
   used a fixed `time_from_start=20ms`, implying a **sustained ~1 rad/s
   (~57 deg/s)** per-joint velocity whenever the QP-internal target lagged
   real position by more than `delta_max` - true almost the entire time
   during real motion, completely overriding the state's intended 15s
   duration. Cutting `delta_max` 20x (`0.01` -> `0.0005`) made the motion
   confirmed slower on retest, but did **not** stop the continuous drift -
   which led to the deeper finding below.
10. **Root cause: the QP-controlled robot model never resyncs to the real
    arm, and nothing used to check that it hadn't drifted.** mc_rtc keeps
    two robot states: `ctl.robot()` (QP-solved, integrates open-loop from
    the solver's own computed accelerations every tick, *regardless* of
    whether real hardware follows) and `realRobot()` (observed, fed from
    real encoders via the `Encoder` `ObserverPipeline`). `delta_max`
    bounded only a *single cycle's* published step - nothing bounded the
    *accumulated* gap between the two over many cycles. Confirmed directly:
    a diagnostic comparing `ctl.robot()`'s joint angles against real encoder
    readings showed multi-radian divergence, growing throughout `MoveHome`
    and never recovering, reproducible with near-identical numbers in both
    a dry-run test and a live test with `delta_max` already cut 20x -
    proving the gap wasn't about `delta_max`'s exact value. **Fixed**
    (`kortex_mc_rtc_bridge_impedance.cpp`, 2026-08-19): the divergence check
    (previously logged only every 500 ticks, informational only) now runs
    every tick and is a hard publish gate - if any joint's
    `|ctl.robot() - real encoder|` exceeds `model_real_gate` (new parameter,
    `0.05` rad), the bridge refuses to publish that cycle and logs a loud,
    rate-limited `SAFETY GATE` error instead of sending a command derived
    from a model that's raced ahead of reality. `delta_max` restored to a
    more usable `0.005` (from the over-corrected `0.0005`) now that the gate
    is the actual safety backstop, not the per-cycle clamp.
11. **A second, deeper instance of the same `ctl.robot()`-vs-real gap, this
    time inside the FSM itself (found 2026-08-21 during a full state-machine
    review, prompted by wanting a hard guarantee that the pipeline always
    reaches real Home before doing anything else).** Independent of the
    bridge-side gate above: every state's own "have I arrived" check
    (`CartesianMove::run()`, `ComplianceCartesianMove::run()`) compared
    `ctl.robot()` against the target - never `realRobot()`. Since the QP
    dutifully drives `ctl.robot()` to the target regardless of whether the
    real arm follows (especially now that the bridge gate above can hold
    off publishing for a while), a state could log "Reached target" and
    hand off to the next state while the real arm was still elsewhere.
    Compounding this, nothing ever resynced `ctl.robot()` back to
    `realRobot()` between states, so a *new* state's trajectory (and any
    `target: {ref: current}` capture, used by `HoldCurrent`) could be built
    from wherever the internal model had drifted to - not from the arm's
    actual pose. This is a strong candidate for the "unexpected direction"
    part of problem 9, independent of pacing. **Fixed**
    (`PickPlaceStates.cpp`): added `resyncControlToReal()`, called at the
    start of `CartesianMove`, `ComplianceCartesianMove`, and `JointMove`,
    which copies `realRobot()`'s `q`/`alpha` onto `ctl.robot()` before
    planning anything new; `resolveTarget()`'s `ref: current` and
    `inherit_orientation` now read `realRobot()` directly; all three
    states' convergence checks now compare against `realRobot()`. Also:
    `CartesianMove` (used by `MoveHome`/`ReturnHome`) no longer force-
    advances to the next state on a settle timeout - it now holds and logs
    loudly instead, since silently proceeding on an unconverged `MoveHome`
    would defeat the entire point of guaranteeing Home before the rest of
    the pipeline runs. `ComplianceCartesianMove` (`MoveToPick`/
    `MoveToPlace`) intentionally keeps its old force-advance-on-timeout
    behavior for now - not changed, flagged as an open question (see Next
    steps) since it weighs differently mid-pipeline with a human nearby.
    **Decided 2026-08-24**: keep force-advancing (a stuck compliant move
    shouldn't hang an experiment session mid-trial), but the log is now
    tagged distinctly - `[MoveToPick] FORCED ADVANCE -> CloseGripper: NOT
    converged...` - so a trial where the grasp/place may not have actually
    happened is greppable in saved session logs instead of reading as a
    normal success.
12. **`home_pose`'s web-app-derived values (problem 8) were wrong in both
    translation and rotation - confirmed and fixed directly, 2026-08-24.**
    Two log-only diagnostics were added first (safe under `dry_run`, no
    behavior change) to investigate why `MoveHome`'s dry-run test showed
    `joint_5` diverging steadily then going perfectly flat at ~1.9-2.4 rad
    partway through the trajectory (a signature of hitting a kinematics
    limit - confirmed `joint_5`'s hard limit is `±2.09 rad` in
    `kinova_6dof_PHYSICALROBOT.urdf` - rather than the task naturally
    converging): (a) the bridge's periodic divergence log now also prints
    the absolute model/real joint values, not just the gap; (b)
    `CartesianMove::start()` now logs all 6 real joint angles from
    `realRobot()`. With the operator confirming the arm was physically at
    Kinova's real Home for a retest, the pre-existing bridge startup log
    (`Joint joint_5 | init_q: ...`) plus `MoveHome`'s own "from" pose log
    gave a direct, mc_rtc-native measurement of true Home - and it did not
    match the web-app-derived `home_pose` at all: translation off by
    11.8cm in X (Z and Y were already close), and rotation's pitch off by
    ~90 degrees (`-89.98°` true vs `+0.11°` configured) - a different
    orientation regime, not a small error. This retroactively explains the
    `joint_5` limit-hitting behavior: the old target asked for a pose that
    apparently needed extreme wrist articulation to reach. Root cause:
    Kinova's web app reports the EE pose at a different reference point
    along the tool than mc_rtc's `tool_frame` (TCP vs flange or similar) -
    copying its numbers directly was never valid, independent of the
    Euler-convention question raised in problem 8. **Fixed**: `home_pose`
    in `PickPlaceController.yaml` now uses the directly-logged mc_rtc-native
    values (`translation: [0.454, 0.001, 0.423]`,
    `rotation: [3.0267, -1.5704, 1.6856]`) - since `CartesianMove`'s
    rotation log already uses the same Euler decomposition
    `poseFromConfig()` recomposes from, these values needed no convention
    translation, unlike the web-app numbers.
    **Dry-run retested 2026-08-24, passed cleanly**: with the arm still
    near real Home, `model-vs-real max joint deviation` stayed tiny the
    whole run (0.0002 -> 0.0017 rad, on `joint_4`, nowhere near
    `model_real_gate`/any joint limit), and both `MoveHome` and
    `ReturnHome` correctly reported real convergence and advanced:
    `[success] [MoveHome] Reached target (pos_err=0.0004 m,
    ori_err=0.0003 rad)` -> `ReturnHome` -> same -> `Idle`. First time the
    full `MoveHome -> ReturnHome -> Idle` sequence has completed cleanly.
    Caveat: the arm started already close to `home_pose`, so this didn't
    exercise a real, meaningful-distance move - doesn't yet rule out
    hitting the same kind of issue over a larger travel distance. **Next**:
    one more dry-run with the arm deliberately moved somewhat away from
    Home first, before any live retest.
    `pick_pose`/`place_pose` were not derived via the web app in the same
    way (chosen as plausible workspace coordinates, not cross-referenced
    against a snapshot) so this specific failure mode is less likely to
    apply to them, but they remain untested regardless (unchanged open
    item, see Next steps).
13. **`MoveHome` stalls partway from a realistic non-Home starting pose -
    switched to joint-space, 2026-08-24.** Dry-run from a pose reached via
    a mix of the Kinova app's pose/angular jog controls (operator confirmed
    not an unusual configuration) showed the same divergence-then-flat
    pattern as problem 12, but this time on `joint_4` and definitively
    *not* a joint-limit repeat: `joint_4` is `continuous` in the URDF (no
    position limit at all) and its velocity never approached its `1.2218
    rad/s` limit either. The absolute-value diagnostics from problem 12
    showed the model's `joint_4` decelerating smoothly (0.199 -> 0.087 ->
    0.0026 rad/s over the last three samples) to a dead stop at
    `model=-0.0238 rad`, while `pos_err`/`ori_err` stayed clearly non-zero
    (0.0975m / 0.2661 rad) - the overall Cartesian task was still far from
    converged, yet this one joint stopped contributing. That smooth-
    decelerate-to-stop shape (not an abrupt clamp) is more consistent with
    the YAML's self-collision constraint (`iDist: 0.15, sDist: 0.05`)
    engaging, or a kinematic singularity in the QP's IK, than a hard bound.
    Also confirmed while investigating: cross-referencing the web app's raw
    actuator degrees (`latest_nonhome.json`) against the bridge's logged
    `joint_i` radians for this same pose matches exactly once wrapped to
    the same range - the joint encoder mapping itself is solid, not a
    factor here. **Not fixed at the root** (would need to directly inspect
    collision distances or QP conditioning, which isn't currently logged
    anywhere, and guessing at that API without the ability to compile here
    risked a wasted round-trip) - instead tried a different-by-construction
    workaround: `MoveHome` switched from `CartesianMove` (Cartesian IK, QP
    picks the joint path) to `JointMove` (direct joint-space interpolation
    to the real Home joint configuration captured in problem 12,
    `[0.0, 0.2622, -2.2691, 0.0, 0.9598, 1.5708]`) - a fundamentally
    different route through configuration space to the same physical
    target. Old Cartesian version preserved commented-out in the YAML.
    **Dry-run retested, 2026-08-24 - result was good, but the test design
    itself was flawed and needed correcting.** Under dry_run the real arm
    structurally cannot move at all (nothing ever publishes), so a
    realRobot()-based convergence check can never show a far-away target
    being reached - `err` (JointMove) and `model-vs-real max joint
    deviation` both stayed exactly pinned at their initial values for the
    entire ~25s log, which is NOT a stall, just this same limitation
    documented elsewhere in this file re: CartesianMove - a mistake to
    re-litigate here since it was already known and momentarily forgotten
    when this specific test was suggested. The USEFUL signal was in the
    absolute values instead: `ctl.robot()`'s worst joint (`joint_6`)
    converged to its target exactly (`model=1.5708`, dead on) within ~2s,
    with no plateau-short-of-target - unlike the Cartesian version, nothing
    here looks stuck on a collision/singularity. That said, reaching target
    in ~2s from a ~0.5 rad error implies a peak commanded rate around 0.88
    rad/s, far exceeding what the bridge's `delta_max` (0.005 rad/cycle
    @100Hz =~0.5 rad/s max) lets the real robot track - `JointMove` has no
    time-parameterized pacing at all (`duration` is only a minimum wait
    before checking convergence, unlike `CartesianMove`'s
    `BSplineTrajectoryTask`), so live this would likely trip the bridge's
    `model_real_gate` almost immediately (safely, but stalling progress).
    **Fixed**: `MoveHome`'s `stiffness` lowered 40x (2.0 -> 0.05) to slow
    its natural convergence rate to an estimated ~0.14 rad/s peak,
    comfortably under `delta_max`'s tracking ceiling - an estimate only,
    since dry-run cannot exercise real tracking to confirm it. **The
    pacing question can only be resolved live from here** - no further
    dry-run test would add information.
14. **First successful live test of `MoveHome -> ReturnHome -> Idle`,
    2026-08-26** - with an additional, unconditional safety margin for this
    specific first attempt: `delta_max` cut further (0.005 -> 0.002,
    ~28.6 -> ~11.5 deg/s hard cap) on top of problem 13's `stiffness: 0.05`,
    so the real robot's speed is capped independent of whether that
    stiffness estimate was right. Arm started at/very near real Home
    (deltas ~0.0001-0.0006 rad on all 6 joints) - by design, to
    minimize risk on the very first live run of this new code path.
    Result: clean end-to-end success - `model-vs-real max joint deviation`
    stayed pinned at 0.0000-0.0002 rad throughout (nowhere near
    `model_real_gate`'s 0.05, which never fired), `MoveHome` converged
    (`err=0.0001`) and handed off to `ReturnHome`, which also converged
    (`pos_err=0.0001m, ori_err=0.0004 rad`) and reached `Idle` cleanly, with
    only noise-level residual wrench afterward. First live confirmation
    that the corrected `home_pose`, the joint-space `MoveHome`, and the
    `model_real_gate` mechanism all work correctly together on real
    hardware - this is the user's original target sequence ("from the
    current pose, go to the home position, then stay there"), now
    live-validated for the first time.
    **Important caveat - not yet a complete validation**: because the arm
    started already at Home, this exercised the pipeline/wiring but NOT the
    actual thing problem 13's retuning was for - real tracking pace and the
    safety gate's behavior under a MEANINGFUL, non-trivial displacement.
    That remains untested live. Also worth noting: a real-hardware
    debugging session hit a false alarm mid-troubleshooting this problem -
    a "did it actually run live" log was misread as still-dry-run when it
    was actually a stale/unrebuilt launch file being used; resolved by
    verifying the installed launch file directly rather than trusting the
    source edit alone. Worth remembering for future debugging: always
    verify what's actually installed and running, not just what's in the
    source tree, when a live/dry-run test's behavior doesn't match
    expectation.

### Current state

- Full FSM defined in `PickPlaceController.yaml`, `Unfold` removed
  (2026-08-19, by request - starts directly from home instead):
  `MoveHome -> MoveToPick -> CloseGripper -> MoveToPlace -> OpenGripper -> ReturnHome -> Idle`.
  `Unfold`'s definition is preserved commented-out in the YAML if needed
  again.
- **`MoveHome`'s `next:` is temporarily bypassed to `ReturnHome`** (not
  `MoveToPick`) to isolate-validate the two rigid legs alone before
  `MoveToPick`/`MoveToPlace` - restore `next: MoveToPick` (transition is
  preserved commented-out in the YAML) once that passes.
- `dry_run: False` in `pick_place_real.launch.py` (as of the 2026-08-26 live
  test, problem 14) - **three live E-stops happened testing `MoveHome`**
  (problems 8-9) before the fixes in problems 10-13 were developed and
  dry-run-validated. First live retest (problem 14, arm started at/very
  near Home) passed completely cleanly: full `MoveHome -> ReturnHome ->
  Idle` sequence, `model_real_gate` never needed to fire, no operator
  intervention needed. **Not yet validated live from a meaningful
  distance** - the actual pacing fix (delta_max/stiffness retuning) and the
  safety gate's real behavior under real divergence remain unexercised.
  Treat the *next* live test (any non-trivial displacement) as needing the
  same fresh E-stop reconfirmation as every `dry_run:false` transition this
  project, even though the flag is already `False` - the risk profile of a
  real, larger live move is materially different from what's been
  confirmed so far.
- **Live-validated** (`dry_run:false`, E-stop operator present):
  - A temporary standalone `HoldCurrent` state (compliant hold at whatever
    pose the arm starts at) - confirmed correct translational yield +
    return-to-position for 2 directions (down, right). Predates, and is
    independent of, problems 8-14.
  - `MoveHome -> ReturnHome -> Idle` (problem 14, 2026-08-26), for an
    arm-starts-at-Home case specifically - clean, full success, no operator
    intervention needed. **Not yet validated for a meaningful displacement**
    - see below.
- **Home-first guarantee**: architecturally, `init: MoveHome` in the YAML
  means every fresh controller start begins by driving to the fixed,
  configured `home_pose` - now (problem 12) derived directly from mc_rtc's
  own forward kinematics with the arm at real Home, not cross-referenced
  against the web app - and, as of problem 11, the state will hold rather
  than silently advancing until `realRobot()` actually confirms arrival.
  Live-confirmed (problem 14) for an arm-starts-at-Home case. **Still not
  confirmed for the actual "come home from wherever you are" case** - the
  scenario the joint-space switch and delta_max/stiffness retuning
  (problems 13-14) specifically targeted remains live-untested: dry-run
  showed clean convergence in the model from a realistic farther-away start
  (problem 13), but dry-run cannot exercise real tracking pace or the
  safety gate under real divergence. This is the current blocker on the
  home-first guarantee actually holding in general, not just near-home.
- **Contact-safety scope deliberately narrowed**: the wrench estimator
  computes a single equivalent wrench as-if applied at the tool frame, so
  contact elsewhere on the arm's body (confirmed live: pushing near the
  base/mid-links) yields an inconsistent response (sometimes negligible,
  sometimes abrupt) rather than a smooth yield. Decided to scope the safety
  guarantee to "participants only ever contact the gripper/end-effector
  area" as an explicit experimental-protocol constraint, rather than build
  whole-arm contact compliance (a materially bigger, different control
  problem - per-joint torque-based reaction, not a single tool-frame wrench
  estimate).
- **Rotational/moment sign check**: never done, only translational.
- Admittance/"free-drive and hold" mode: scoped, not built. Architecturally
  small (same `ImpedanceTask`, `stiffness` near zero instead of the spring-
  back values above), but real open items before it's safe: workspace/pose
  limit clamps become load-bearing without a spring pulling back (currently
  only `ImpedanceTask`'s generous default `deltaCompPoseLinLimit_`/
  `AngLimit_`, no explicit Z-floor clamp during free-drive), and `qd_gate`
  revalidation matters more here (sustained hand-guided motion, not brief
  pushes).

### Next steps

1. ~~Resolve the active-motion dry-run tracking-failure investigation~~
   **Done** - confirmed as the `ctl.robot()`-vs-real gap (problems 10-11),
   not a controller bug; both a bridge-side publish gate and an FSM-side
   resync/realRobot()-based convergence check are now in place.
2. ~~Live retest `MoveHome -> ReturnHome` (arm starting at Home)~~ **Done,
   passed cleanly** (problem 14, 2026-08-26) - full sequence completed live
   with no operator intervention, `model_real_gate` never needed to fire.
3. **Live-test `MoveHome` from a meaningful, non-trivial distance** - the
   actual target of problems 13-14's fixes, still completely unexercised.
   E-stop operator present, freshly re-confirmed immediately before this
   specific test even though `dry_run` is already `False` (this is a
   materially different, higher-risk test than problem 14's near-home
   case - same standing protocol as every prior `dry_run:false`
   transition). Move the arm to a real, meaningful distance from `home_pose`
   first (via the web app, similar to or larger than problem 13's dry-run
   test pose), then run. Watch for:
   - `[KortexBridge] SAFETY GATE ... NOT publishing` firing and holding -
     not a failure, the intended fail-safe if divergence grows too fast;
   - `[MoveHome] NOT converged ... holding` similarly - both mean the arm
     stops and holds rather than completing, which is safe but means
     `stiffness: 0.05` and/or `delta_max: 0.002` still need tuning
     (probably `stiffness` lower still, since `delta_max` is the harder
     unconditional cap and shouldn't need to move first);
   - actual smooth, slow progress toward Home within the expected ~11.5
     deg/s ceiling - the success case.
   If this passes, loosen `delta_max` back toward `0.005` (problem 14's
   note) and consider re-testing once more before treating `MoveHome` as
   solved in general.
4. Once step 3 passes: decide whether to restore `MoveHome`'s `next:` to
   `MoveToPick` (revert the temporary bypass) and live-validate
   `MoveToPick`/`MoveToPlace` in isolation next (moving-target compliance,
   never validated live), same staged philosophy as `HoldCurrent`, before
   the full chain.
5. ~~Open question from problem 11~~ **Decided (2026-08-24)**:
   `ComplianceCartesianMove` (`MoveToPick`/`MoveToPlace`) keeps force-
   advancing to the next state on a settle timeout even if `realRobot()`
   never actually converged (unlike `CartesianMove`/`MoveHome`, which now
   holds) - a stuck compliant move shouldn't hang an experiment session
   mid-trial. The forced-advance log is now tagged distinctly
   (`FORCED ADVANCE -> <next state>`, greppable) so a trial where the grasp/
   place may not have actually happened is visible in saved session logs
   instead of reading as an ordinary success - filter session logs for this
   string when reviewing trial data.
6. Complete the sign check: remaining translation directions (up, left) and
   rotational/moment axes - ideally against `MoveToPick`/`MoveToPlace`
   directly, not just the (now superseded) `HoldCurrent` state.
7. Re-validate `wrench_dls_lambda2`/`max_force_estimate`/
   `max_moment_estimate`/`qd_gate_low`/`qd_gate_high` against real, moving
   trajectories - still outstanding since 2026-07-27.
8. Decide whether the arm needs to be manually positioned near `home_pose`
   before launch, or whether `MoveHome` is expected to safely reach it from
   anywhere - relevant again now that `CartesianMove` will hold rather than
   advance if it can't.
9. Confirm gripper actuation is real, not a stub/timeout fallback - every
   dry-run log so far shows `[Gripper] timeout (Ns), proceeding` for both
   close and open rather than a confirmed action-server success; worth
   checking directly once live.
10. Commit the accumulated changes - still uncommitted, see "Branches" above.

## Novelty direction (from the 2026-07-27 discussion)

Chosen direction: **personalization via the same teacher/student
distillation machinery**, not just modality reduction. Teacher (physio +
visual + audio, training-time only) both (a) trains a deployable
contactless/visual-only student and (b) provides a population-level prior
that a fast per-participant calibration step adapts - two contributions
(cross-modal distillation + cross-subject personalization) from one
architecture, which also directly targets the proposal's own observation
that individual comfort baselines vary a lot.

**Contactless emotional-state prediction (checked feasibility, looks solid):**
- Camera-based rPPG (heart rate/HRV from facial video) is mature and
  increasingly robust to motion/illumination; there's dedicated 2025-2026
  work specifically on illumination-robust rPPG *for physiological sensing
  in robots*, i.e. this exact use case.
- Thermal imaging is a validated contactless proxy for arousal/stress
  (perinasal/palm temperature tracks sympathetic activation), used in place
  of GSR in recent multimodal stress-detection work (e.g. one 2024 framework
  combining facial expression + rPPG + thermal reported 97% stress-detection
  accuracy) - if you're open to an extra (still non-contact) thermal camera,
  not just RGB.
- A dataset specifically for this direction already exists: **CAST-Phys**
  ("Contactless Affective States Through Physiological signals"), worth a
  look alongside MultiPhysio-HRC.
- Practical caveat: rPPG needs a reasonably stable, well-lit view of the
  face - since your task moves the participant's hands/arms but not
  (necessarily) their head, this should be manageable, but camera placement
  and lighting consistency become real experimental-design constraints,
  not incidental details.
- Suggested framing: keep contact sensors (HR+GSR) as the training-time
  teacher for best accuracy, but make the deployable/real-time modality set
  richer than plain facial-expression AUs - add camera-derived rPPG-HR/HRV
  (and thermal-derived arousal, if you add that camera) to the student's
  inputs. Closer in information content to the teacher, still fully
  contactless.

## Data strategy

[MultiPhysio-HRC](https://automation-robotics-machines.github.io/MultiPhysio-HRC.github.io/)
(CC-BY-4.0, `Robotics` 2025) is a good resource for de-risking the
perception/distillation pipeline before spending participant sessions on it
- see `research/multiphysio_poc/` for a working proof of concept (teacher on
HRV/EDA/EEG features, student on facial Action Units, both with and without
distillation). It does **not** replace your own trials: it's an open-loop
disassembly task with no adaptive robot behavior and no systematic
trajectory/velocity manipulation, so it can't answer the actual research
question (does adapting motion to inferred comfort improve comfort). Revise
the proposal's "no existing dataset" framing to something sharper: existing
multimodal HRC datasets are passive/open-loop; this project is the first to
pair this sensing stack with a compliant, comfort-adaptive controller and a
controlled trajectory/velocity manipulation.

## Useful commands

```bash
# Build (from the workspace root containing src/)
colcon build --packages-select impedance_control pick_and_place

# Real robot (impedance_control only, single held pose)
ros2 launch impedance_control impedance_real.launch.py

# Gazebo (impedance_control only)
ros2 launch impedance_control impedance_gazebo.launch.py

# Real robot (full pick-and-place FSM, pick_and_place branch)
ros2 launch pick_and_place pick_place_real.launch.py

# Gazebo (full pick-and-place FSM, pick_and_place branch)
ros2 launch pick_and_place pick_place_gazebo.launch.py

# MultiPhysio-HRC distillation PoC
cd research/multiphysio_poc
pip install -r requirements.txt
python prepare_dataset.py --data-dir ./data
python train_distill.py --data-dir ./data
```

## Open items / suggestions for next session

- ~~Compile and bench-test the impedance_control fixes on real hardware~~ /
  ~~compile and test the `pick_and_place` branch, tune
  `contact_force_threshold`/`clear_hold_time` against real push tests~~ -
  **done 2026-08-19**, see the pick_and_place validation session above for
  what's confirmed vs. still open (the "Next steps" list there is now the
  current one to work from).
- Decide whether to commit the two branches' work as-is, and whether the
  `research/multiphysio_poc` ML code belongs on its own branch (or `main`)
  rather than riding along on `pick_and_place`, since it's unrelated to the
  robot control code.
- Improve the distillation PoC per its own README's "why the numbers are
  weak" section (per-participant normalization is the most likely fix, and
  doubles as a first step toward the personalization direction above).
- Look at CAST-Phys as a second existing dataset for the contactless
  direction, alongside MultiPhysio-HRC.

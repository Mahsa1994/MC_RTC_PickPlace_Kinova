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

1. Build and read the code changes - **none of this was compiled or tested
   in this session** (no mc_rtc install available in the environment it was
   written in). Check for typos/API mismatches first.
2. Launch with `dry_run: true` (the current default). Manually push the arm
   in every direction, watch the logged `force_sensor`/`moment_sensor`
   values, and confirm the sign makes sense (pushing the arm should read as
   a force in the direction you pushed, not the opposite).
3. Only then set `dry_run: false`, starting with `delta_max` small (already
   0.01 in the launch file) and someone at the E-stop.
4. Re-validate `wrench_dls_lambda2`, `max_force_estimate`,
   `max_moment_estimate`, `qd_gate_low`, `qd_gate_high` against that data -
   all are exposed as ROS parameters (no recompile needed to retune).

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

Same "not compiled, not tested on hardware" caveat as above applies here too.

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

- Compile and bench-test the impedance_control fixes on real hardware,
  following the verification procedure above, before trusting them.
- Compile and test the `pick_and_place` branch in Gazebo first, then real
  hardware; tune `contact_force_threshold`/`clear_hold_time` against actual
  push tests.
- Decide whether to commit the two branches' work as-is, and whether the
  `research/multiphysio_poc` ML code belongs on its own branch (or `main`)
  rather than riding along on `pick_and_place`, since it's unrelated to the
  robot control code.
- Improve the distillation PoC per its own README's "why the numbers are
  weak" section (per-participant normalization is the most likely fix, and
  doubles as a first step toward the personalization direction above).
- Look at CAST-Phys as a second existing dataset for the contactless
  direction, alongside MultiPhysio-HRC.

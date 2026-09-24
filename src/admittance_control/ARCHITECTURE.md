# Kinova Gen3 6DOF Joint-Space Hand-Guiding — Architecture & Physics

This document explains how `admittance_control` makes the Kinova Gen3 6DOF
freely hand-guidable (zero-g, "push it and it moves; let go and it freezes")
without any force/torque sensor, using only the built-in joint-torque signal
and standard ROS 2 + mc_rtc components. It is written so someone with basic
ROS 2 knowledge — but no prior exposure to this repo — can rebuild the system
from scratch, or debug it when a joint feels stiff or chattery.

It complements `Readme.md` (older, describes an earlier AdmittanceTask-based
design that this package no longer uses) and `CHANGES_2026-09-24.md`
(chronological changelog with tuning tables). Read this one first if you want
to understand *why* the code looks the way it does; read the changelog for
*what changed and when*.

---

## 1. The problem this solves

Kinova ships a "zero-g" / admittance mode in its own firmware (the Web App /
Kortex API `SetCartesianWrenchCompensationMode` and friends). That mode is
not usable here because **the requirement is that admittance runs through
ROS 2**, so the motion law is inspectable, tunable, and loggable from the
Linux side rather than a black box inside the arm's firmware.

Doing that from ROS 2/mc_rtc runs into three hardware facts about this arm
that shape every design decision below:

1. **No force/torque sensor at the wrist.** The only sensed quantity is
   *per-joint motor torque*, published on `/joint_states.effort` by the
   `ros2_kortex` driver.
2. **The driver only accepts position (or velocity) commands.** Torque/effort
   *command* interfaces are not implemented in `ros2_kortex`'s
   `hardware_interface.cpp` — the arm is always low-level-position-servoed
   from ROS 2's point of view, even though the actuators are torque-capable
   internally. So "admittance" here cannot mean "compute a force and command
   a torque"; it must mean "compute a velocity and command a stream of
   positions that realizes it".
3. **Continuous joints wrap.** joint_1, joint_4 and joint_6 have no mechanical
   end stop and the driver reports their position wrapped to `[-pi, pi]`. Any
   code that compares or integrates these angles naively will see phantom
   2*pi jumps.

Everything in this package is a consequence of those three constraints.

---

## 2. System overview

```
┌────────────────────┐  /joint_states (position, velocity, effort)
│  ros2_kortex driver │ ───────────────────────────────────────────┐
│  (real robot or     │                                            │
│   fake_hardware)    │ <── /joint_trajectory_controller/           │
└────────────────────┘        joint_trajectory (position+velocity) │
         ▲                                                         ▼
         │                                          ┌───────────────────────────┐
         │ JointTrajectory @ ~200 Hz                 │ kortex_mc_rtc_bridge_      │
         └────────────────────────────────────────── │ admittance  (ROS 2 node)  │
                                                       │  - inverse dynamics       │
                                                       │  - per-joint tau_ext      │
                                                       │  - angle-wrap-safe I/O    │
                                                       │  - safety gates           │
                                                       └─────────────┬─────────────┘
                                                                     │ owns / drives
                                                                     ▼
                                                       ┌───────────────────────────┐
                                                       │ mc_rtc::MCGlobalController │
                                                       │  FSM controller            │
                                                       │  "KinovaHandGuiding"       │
                                                       │                            │
                                                       │  Init → HoldPosition        │
                                                       │        ⇅ (GUI button)       │
                                                       │       HandGuide             │
                                                       │        → UpdateHoldTarget   │
                                                       └───────────────────────────┘
```

Two processes, both under ROS 2:

- **`kortex_mc_rtc_bridge_admittance`** (`src/kortex_mc_rtc_bridge_admittance.cpp`):
  a `rclcpp::Node` that (a) turns raw joint effort into a clean per-joint
  external-torque estimate, (b) drives the mc_rtc control loop at a fixed
  rate, and (c) streams the FSM's output positions to the trajectory
  controller, with several safety gates in between.
- **The mc_rtc FSM controller `KinovaHandGuiding`**
  (`src/KinovaHandGuiding.cpp` + `src/states/*.cpp`): owns the admittance law
  itself, running as a library loaded *inside* the bridge process via
  `mc_control::MCGlobalController` (not a separate node — mc_rtc is a
  library, not a ROS node).

Everything downstream of the bridge is the standard ROS 2 control stack:
`joint_state_broadcaster` + `joint_trajectory_controller`
(`JointTrajectoryController`, position command interface) from
`ros2_control`, spawned by the stock Kortex `gen3.launch.py`. This package
does not replace or modify that stack — it only talks to it as a normal
client over `/joint_states` and `/joint_trajectory_controller/joint_trajectory`.

---

## 3. Why mc_rtc, and why an FSM

mc_rtc is used as the task-space/joint-space solver: it turns "track this
joint velocity" into a QP-consistent joint position/velocity command while
respecting joint limits, velocity limits and collision-avoidance constraints
declared in `KinovaHandGuiding.yaml`. Hand-rolling that integration would
mean re-deriving joint-limit clamping, and the FSM/PostureTask machinery
already does it.

The controller is a 4-state FSM (`etc/KinovaHandGuiding.yaml`):

```yaml
transitions:
  - [Init,              OK,             HoldPosition,     Auto]
  - [HoldPosition,      startGuiding,   HandGuide,        Auto]
  - [HandGuide,         stopGuiding,    UpdateHoldTarget, Auto]
  - [UpdateHoldTarget,  OK,             HoldPosition,     Auto]
```

- **`Init`**: built-in `MetaTasks` state, exists for exactly one control
  cycle to let mc_rtc finish constructing the robot before any custom state
  touches it.
- **`HoldPosition`** (`HoldPositionState.cpp`): a stiff `PostureTask`
  (stiffness 10, weight 100) holding the arm still. Exposes a GUI button
  "Start Hand-Guiding" that transitions to `HandGuide`.
- **`HandGuide`** (`HandGuideState.cpp`): the admittance law itself — see
  §5. Exposes "Stop Hand-Guiding".
- **`UpdateHoldTarget`** (`UpdateHoldTarget.cpp`/`.h`): a one-cycle state that
  re-seeds the next `HoldPosition`'s `PostureTask` target from the arm's
  *actual, encoder-observed* pose (`ctl.realRobot().mbc().q`) instead of the
  pose the controller happened to start at. Without this state the arm would
  snap back toward the controller's startup pose the moment you release it.

`HandGuide` internally has its own hold/guide sub-behavior driven by torque
magnitude (§5.4) — the FSM-level `HoldPosition`/`HandGuide` split exists so a
human operator can explicitly arm/disarm hand-guiding from the GUI, e.g.
before letting go of the arm entirely.

---

## 4. The bridge: estimating "how hard is someone pushing?"

### 4.1 Two robot models, one purpose each

`MCGlobalController` keeps two `Robot` objects, and getting this distinction
right is the single most important thing in the bridge:

- **`gc_->realRobot()`** — updated every tick straight from encoder readings
  (`gc_->setEncoderValues/Velocities/JointTorques`). This is "where the arm
  physically is right now".
- **`gc_->robot()`** — the *control* robot: the FSM/QP's own open-loop state,
  integrated forward each `gc_->run()` call from the solver's commanded
  accelerations. It is never resynchronized to the encoders. This is "where
  the QP thinks it has commanded the arm to be".

Rule used throughout the bridge: **any physics computation (gravity,
inertia, torque residual) uses `realRobot()`; only the final published
command reads from `robot()`.** Using the control robot for the physics was
tried early on and produces a positive-feedback failure: if the model ever
gets even slightly ahead of the real arm, gravity comp is evaluated at the
wrong configuration, the resulting residual is misread as an external push,
that push is integrated into more commanded motion, and the model runs away
further — this produced the "three live E-stops" mentioned in the source
comments before the fix.

### 4.2 Estimating external joint torque (the "sensor" this arm doesn't have)

There is no wrist F/T sensor, so the bridge estimates, every control tick,
"how much of the measured motor torque is *not* explained by gravity,
Coriolis and the arm's own commanded acceleration". What's left over is
attributed to a human pushing on the arm.

Step by step (`controlLoop()`):

1. **Read measured torque** from `/joint_states.effort`
   (`enc_tau` → `latest_efforts_` → `tau_meas`, mapped into RBDyn's DOF
   ordering via `buildJointDofMap`).

2. **Estimate joint acceleration.** RBDyn's inverse dynamics needs `qddot` to
   separate "torque needed to accelerate the arm" from "torque from an
   external push". The obvious approach — finite-difference the encoder
   velocity every tick — is far too noisy on its own (encoder velocity is
   already somewhat noisy; differentiating it again amplifies that). So it's
   low-pass filtered first:
   ```
   raw_qddot   = (qdot[k] - qdot[k-1]) / dt
   filt_qddot += (dt / (dt + accel_filter_tau)) * (raw_qddot - filt_qddot)
   ```
   with `accel_filter_tau = 0.1 s` (a first-order IIR low-pass, exact
   discretization of `tau * xdot + x = input`). Before this filter existed,
   `qddot` was hard-coded to zero, which meant *any* real acceleration —
   including the tiny corrective jitter the QP makes while "holding still"
   — was misattributed to an external push, causing jerkiness with nobody
   touching the arm.

3. **Run RBDyn inverse dynamics** (`rbd::InverseDynamics::inverseDynamics`)
   at `realRobot()`'s current `q`, `qdot`, and the filtered `qddot`, with
   gravity `(0,0,9.81)`. This gives `tau_bias`: the torque each joint's motor
   *should* be producing right now if there were no external contact — i.e.
   gravity + Coriolis/centrifugal + the joint's own commanded-acceleration
   torque.

4. **Add payload gravity compensation.** The mc_kinova URDF stops at
   `bracelet_link` and knows nothing about the mounted Robotiq 2F-85
   gripper. It is modelled as a point mass (`payload_mass`, default 0.9 kg)
   offset from `bracelet_link` by `payload_com` (default `[0,0,-0.12]` m,
   roughly the gripper's center of mass beyond the flange at
   `z = -0.0615`). Its contribution to joint torque is the standard
   Jacobian-transpose static force mapping:
   ```
   tau_bias += J_lin(payload_point)^T · (0, 0, m·g)
   ```
   built once as an `rbd::Jacobian` at bridge init and re-evaluated (not
   re-allocated — see §7.1) every tick via `fullJacobian`.

5. **Subtract:** `tau_ext = tau_bias - torque_sign * tau_meas`. `torque_sign`
   exists because different firmware/driver combinations report motor
   torque with different sign conventions relative to RBDyn's; it is a
   single ROS parameter (`-1.0` on this hardware) rather than something
   baked into the formula.

6. **Startup delay + dynamic tare.** For the first 3 seconds after the
   bridge connects, `tau_ext` is forced to zero (position/velocity encoder
   readings are still settling and the acceleration filter hasn't converged,
   so early residuals are noisy). For the following 0.5 s, the residual is
   averaged into a per-joint bias vector `bias_tau_` instead of being used —
   this captures whatever small, consistent modeling error is left
   (URDF mass/CoM inaccuracies, encoder offset, etc.) and subtracts it out,
   the same idea as taring a scale. After that, every tick does
   `tau_ext -= bias_tau_`.

7. **Low-pass + soft deadband.** The tared residual is smoothed with another
   50 ms IIR filter, then passed through a *soft* deadband:
   ```cpp
   softDeadband(v, db) = |v| <= db ? 0 : v * (|v| - db) / |v|
   ```
   instead of a hard `if (|v| < db) v = 0`. A hard cutoff is discontinuous
   exactly at the threshold that matters most: at 0.49 Nm you get nothing, at
   0.51 Nm you get the full 0.51 Nm — so a push that hovers near the
   threshold makes the joint chatter (move → measured torque relaxes below
   threshold → stops → push resumes → moves...). The soft version keeps the
   same "below `db`, output is exactly zero" guarantee but removes the step.

8. **Per-joint velocity gate.** Each joint's own residual is scaled down while
   *that joint* is already moving fast:
   ```
   gate = 1 - clamp((|qdot| - qd_gate_low) / (qd_gate_high - qd_gate_low), 0, 1)
   ```
   linear from 1 at `|qdot| <= qd_gate_low` (0.05 rad/s) down to 0 at
   `|qdot| >= qd_gate_high` (0.3 rad/s). This rejects leftover inertial/
   tracking-error torque that the acceleration filter's lag didn't fully
   cancel, without needing a whole-arm "is anything moving" cutoff — a
   design change from an older Cartesian version of this bridge, where any
   joint's motion suppressed compliance everywhere.

9. **Comms watchdog.** If `/joint_states` hasn't arrived in >100 ms,
   `tau_ext` is force-zeroed *before* publishing/handing it to mc_rtc (an
   earlier version zeroed it only after publishing, which was dead code).

The result, `tau_ext` (one value per joint, Nm), is written to
`gc_->controller().datastore()` under the key `"KHG::tau_ext"` — this is how
the bridge hands data to the FSM state without either one depending on the
other's headers; both only agree on a string key and an `Eigen::VectorXd`
type. It's also republished as `/admittance/tau_ext` (post filter/deadband)
and `/admittance/tau_ext_raw` (pre) for external logging/debugging.

### 4.3 Driving the control loop at the right rate

`gc_->run()` integrates the QP's internal state assuming exactly
`Timestep` seconds pass per call, *regardless of real elapsed time*. The
bridge therefore creates its `rclcpp::TimerBase` with
`gc_->timestep()` (read from the loaded config, 0.005 s = 200 Hz here), not a
hardcoded value — calling `run()` faster than that silently makes the
internal model race ahead of real time (e.g. a "10 s" motion finishing in
2 s of wall time), which previously produced a runaway QP divergence and a
segfault. `publish_rate` (default 200 Hz, same as the control rate here) lets
the *published* command rate be decimated relative to the control rate if
the control rate is ever raised without wanting to flood the trajectory
controller; `pub_decim_ = round(1 / (loop_dt * publish_rate))`.

### 4.4 Publishing: angle wrap and the safety gates

This is where fact #3 from §1 (continuous joints wrap to `[-pi, pi]`) has to
be handled explicitly, because mc_rtc's internal `q` for a continuous joint
is **not** wrapped — it just keeps counting past pi. Comparing it directly to
the encoder's wrapped reading is wrong by design once the joint passes 180°:
e.g. model `q = 3.20 rad`, encoder `q = -3.14 rad` describe the same physical
angle but differ by ~6.3 rad if subtracted naively.

Every model-vs-encoder comparison in the bridge therefore goes through:

```cpp
static double angleDiff(double a, double b) {
  return std::remainder(a - b, 2.0 * M_PI);   // result in (-pi, pi]
}
```

`std::remainder` (not `fmod`) is what gives the smallest-magnitude signed
difference, which is what "how far apart are these two angles" should mean.

Three places use it, all inside the per-tick publish path:

1. **First-command sanity check.** Before ever publishing, verify the FSM's
   very first commanded position for each joint is close
   (`|angleDiff| <= 0.05 rad`) to where the encoder already is. If not,
   nothing is published and an error is logged — this catches a
   misconfigured `stance`/initial seed before it ever reaches the arm.
2. **Per-cycle command clamp.** The published position is *always* expressed
   relative to the current encoder reading, not the model's raw `q`:
   ```cpp
   diff  = angleDiff(q_cmd_model, q_encoder);
   q_cmd = q_encoder + clamp(diff, -delta_max, +delta_max);
   ```
   `delta_max` (default 0.02–0.05 rad depending on launch config) bounds how
   far a single published point can be from the arm's actual current
   position — a hard limit on commanded velocity between two publish ticks,
   independent of anything mc_rtc computed. Because the result is built as
   "encoder + small clamped delta", it is automatically expressed in the
   encoder's own wrapping and the trajectory controller never sees a raw
   `2*pi` jump.
3. **Safety gate (`model_real_gate`, default 0.05 rad).** Every tick, the
   worst-case `|angleDiff(model_q, encoder_q)|` over all 6 joints is
   computed. If any joint's model and encoder have drifted apart by more
   than this, **no command is published at all** until it clears. This is
   the last line of defense against the control-robot-runs-away failure mode
   from §4.1 — if the model is meaningfully wrong about where the arm is, the
   safest thing is to stop commanding rather than trust it.

Before `angleDiff` existed, this gate itself was the source of a bug: turning
joint_1 (a continuous joint) past ±180° made the model (`3.17 rad`, still
counting) and the wrapped encoder (`-3.14 rad`) look 2*pi apart, tripping the
gate and freezing the arm exactly when the operator was doing nothing wrong.

If `dry_run` is true (the default in `admittance_real.launch.py` until
explicitly overridden), everything above still runs and logs, but the final
`pub_->publish(traj)` is skipped — useful for watching the estimator/gates
behave correctly before ever moving the real arm.

### 4.5 Command message shape

The published `trajectory_msgs::msg::JointTrajectory` has exactly one point,
re-sent every publish tick (a "rolling setpoint" pattern, not a multi-point
trajectory):

```cpp
pt.positions  = q_cmd (6 values, computed as above)
pt.velocities = gc_->robot() joint velocities (feeds JTC's cubic interpolation)
pt.time_from_start = 2 * loop_dt * pub_decim_   // always >= 2 publish periods ahead
```

Sending both position *and* velocity lets `joint_trajectory_controller`
(configured with `open_loop_control: true` in `config/ros2_controllers.yaml`)
interpolate smoothly between successive setpoints instead of chasing a
staircase of pure position waypoints. `time_from_start` is deliberately kept
at least two publish periods in the future so there is always an
unexpired segment for the controller to interpolate into, even if one publish
is delayed.

On node shutdown (`~KortexMcRtcBridge`), a final single-point trajectory is
published holding the arm at its last known real (encoder) position with
zero velocity, then the node sleeps 100 ms before letting the socket close —
otherwise the arm can be left mid-command if the process exits while a
trajectory is still "in flight".

---

## 5. The FSM state: from torque to motion (`HandGuideState.cpp`)

This is the actual admittance law. Everything upstream (§4) exists to feed
this state one clean number per joint (`tau_ext`, via the datastore); this
state's only job is: given `tau_ext`, decide `qdot_desired`, and get the QP
to realize it.

### 5.1 Why PostureTask + `refVel`, not AdmittanceTask/ImpedanceTask

mc_rtc ships an `AdmittanceTask` (Cartesian, needs a real F/T sensor) and
`ImpedanceTask` (has a spring term that pulls back toward a reference pose —
wrong here, hand-guiding must *not* spring back). Neither fits: there is no
Cartesian wrench (only per-joint torque, §4.2) and no F/T sensor to feed
`AdmittanceTask` with. A plain `mc_tasks::PostureTask` is used instead,
driven in **velocity**, not just position:

```
qddot_cmd = K * (q_target - q) + D * (refVel - qdot)
```

(This is Tasks' standard PD-with-reference-velocity law, not something custom
to this package.) The state sets, every tick:

- `target = q_ctl + qdot_des * dt` — one Euler step ahead of the *control*
  robot's own current position, not the measured one.
- `refVel = qdot_des` — the same desired velocity fed as the D-term's
  reference.

With this pairing, `target - q_ctl ≈ qdot_des * dt`, so the position term
mostly just reinforces the velocity term instead of fighting it; the net
effect is the control robot's velocity converges to `qdot_des` with time
constant `1/D`. **Anchoring `target` to `q_ctl` (the control robot) rather
than the measured/encoder pose is deliberate and important:** the real arm is
position-controlled downstream and therefore always lags the commanded
position by some tracking error. If `target` were anchored to the measured
pose instead, it would sit roughly one tracking-error *behind* the model on
every tick, and the QP — seeing its own state already "ahead" of a target
that keeps resetting behind it — would never accumulate net motion. This was
the original "arm is rigid, does not move at all when pushed" failure mode
before this fix.

### 5.2 Precomputing per-joint reflected inertia (`H_ii`)

At `start()`, the state builds an `rbd::ForwardDynamics` for the robot and
computes the joint-space mass matrix `H(q)` once at the arm's configured
`stance` pose (`mc_rtc.yaml`'s `kinova_6dof.stance`, a folded/neutral
posture), storing each joint's diagonal `H_ii` as `hRef_[i]`. `H`'s diagonal
entries are each joint's *reflected inertia* — how much apparent mass that
joint has to move, which for a serial arm depends strongly on the whole
current configuration (e.g. shoulder inertia as seen at joint_2 is very
different with the arm folded vs. fully extended). This reference value is
reused every tick in §5.4.

### 5.3 The admittance law

Per joint `i`, every control tick (`dt` = solver timestep):

```
tau_in       = holding ? 0 : tau_ext[i]
qd_raw       = clamp(admittance[i] * inertiaScale[i] * tau_in, -maxJointVel, +maxJointVel)
qdot_f[i]   += lag * (qd_raw - qdot_f[i])         // first-order lag, "virtual inertia"
qdot_des[i]  = qdot_f[i]
```

with `lag = dt / (velocityTau + dt)`. This is a chain of three deliberate
design choices, each added to fix a specific observed failure mode:

- **`admittance[i]` (rad/s per Nm)** is the base gain: how fast should this
  joint move per unit of estimated external torque. Set per-joint in
  `KinovaHandGuiding.yaml`, larger for the wrist joints (0.08) than the
  shoulder/elbow (0.05) because a given hand force produces less torque at a
  joint the further out it acts.
- **`inertiaScale[i]` — configuration-dependent gain compensation.** Computed
  every tick when `inertiaScaling: true`:
  ```
  H = ForwardDynamics::computeH(mb, ctl.robot().mbc())   // current mass matrix
  inertiaScale[i] = clamp(hRef[i] / H_ii(q), inertiaScaleMin, inertiaScaleMax)
  ```
  The physical motivation: a fixed admittance gain implicitly assumes a fixed
  relationship between applied torque and resulting velocity, but that
  relationship is `qdot ~ tau / H_ii(q)`, and `H_ii` changes a lot across the
  workspace (largest with the arm extended). Without this scaling, the
  *effective* loop gain (tau_ext → commanded velocity → arm accelerates →
  more sensed "torque" from tracking error → ...) grows wherever `H_ii` is
  smallest, i.e. exactly where the arm is lightest/most extended — this was
  observed as a 6–7 Hz bang-bang limit cycle on joint_2 with the arm fully
  vertical. Scaling gain by `H_ii(stance)/H_ii(q)` keeps the loop gain
  roughly configuration-independent. Clamped to `[0.15, 1.0]` so it can only
  ever reduce gain relative to the stance reference, never amplify it beyond
  the tuned baseline.
- **`velocityTau` — first-order lag ("virtual inertia").** Even with
  inertia scaling, directly commanding `qd_raw` reacts to the *current*
  `tau_ext` sample, which is itself the output of an estimator with ~60 ms of
  filter lag (§4.2) riding on encoder noise. Feeding that straight into a
  velocity command closes a loop whose bandwidth is set entirely by how fast
  `tau_ext` changes — too fast for the mechanical system to track cleanly,
  which is the other half of the extension-oscillation story. `qdot_f`
  low-pass filters the raw command with time constant `velocityTau` (default
  0.15 s), which is physically "as if the joint had extra rotational inertia
  it must accelerate", rolling the loop's effective gain off above roughly
  1 Hz and damping the oscillation. Combined with inertia scaling, RMS
  high-frequency joint velocity at full extension dropped from 0.173 to
  0.023 rad/s (measured from logged data, see `CHANGES_2026-09-24.md`).
- **`maxJointVel`** is a hard velocity cap (rad/s) applied before the lag, an
  independent safety bound regardless of how large a torque reading spikes
  to.

### 5.4 Hold/guide hysteresis and "freeze on release"

A single scalar, `tauNorm = max_i |tau_ext[i]|`, drives a hysteretic
hold/guide switch:

```
holding=true  AND tauNorm > guideTorque (1.0 Nm) → holding=false ("GUIDING")
holding=false AND tauNorm < holdTorque  (0.5 Nm) → holding=true  ("HOLD")
```

Hysteresis (two different thresholds, `guideTorque > holdTorque`) prevents
chatter right at a single crossing point, the same reasoning as the soft
deadband in §4.2 but at the whole-arm level rather than per-joint.

While `holding`, `tau_in` in the law above is forced to zero for every
joint — the commanded velocity does **not** snap to zero, it *decays through
the same lag* (`qdot_f` relaxes toward 0 with time constant `velocityTau`),
so releasing the arm mid-motion produces a smooth stop rather than an abrupt
one.

Once holding *and* the filtered velocity has actually settled
(`max|qdot_f| < 1e-3 rad/s`), the state latches `hold_q_` to the control
robot's current position on the first settled tick, zeroes everything, and
calls `postureTask_->posture(hold_q_)` — from then on it is a plain stiff
position hold at exactly the pose the arm coasted to, not the pose it was at
the instant `holding` became true. This is what makes "let go and it freezes
in place" true rather than "let go and it snaps back".

### 5.5 Live tuning via the mc_rtc GUI

`start()` registers, under a GUI category `Control`: a Stop button, a
hold/guide state label, the live `max|tau_ext|` readout, editable hold/guide
thresholds, an editable 6-entry admittance array, max velocity, velocity lag
time constant, an inertia-scaling checkbox, a read-only applied-inertia-scale
array, and editable posture stiffness/damping. All of these are the same
variables read from YAML at `configure()`/`start()` time — the GUI just gives
write access to the live values, so gains can be tuned on the running robot
without a rebuild (see `CHANGES_2026-09-24.md` for which parameters need a
YAML edit + rebuild vs. which are GUI-live). Log entries
(`HandGuide_tau_ext`, `_qdot_des`, `_holding`, `_inertia_scale`, `_H_diag`)
are added so every quantity in this section is visible afterward in the
mc_rtc binary log.

---

## 6. Robot model: the `Kinova6DOF` mc_rbdyn module

mc_rtc needs a `mc_rbdyn::RobotModule` describing the arm's kinematics,
dynamics and joint limits. This comes from `mc_kinova`
(`isri-aist/mc_kinova`), which upstream only ships a 7-DOF Gen3 variant. The
6-DOF support (`Kinova6DOF` module name, `kinova_6dof` URDF/RSDF/convex-hull
variant) is a **local patch on top of that upstream repo**, not part of this
ROS 2 package's own build — see `patches/mc_kinova/README.md` for the patch
itself and how to (re)apply it. In short, it:

- registers a new module name `Kinova6DOF` that loads the `kinova_6dof`
  URDF/RSDF/convex variant instead of the 7-DOF default, and skips every
  `joint_7`-specific override;
- fixes joint limits for the two wrist joints that are true infinite-rotation
  actuators on the 6-DOF arm (joint_4, joint_6 — `continuous` in the URDF,
  no mechanical stop) but were previously left clamped to the 7-DOF arm's
  values (±2.45 rad / ±2.0 rad) by the original patch. joint_1 was already
  correctly unbounded; joint_5 keeps its real ±2.09 rad physical limit
  (spherical wrist 2 does have a mechanical stop on this arm).

This module is what both the bridge (via `MCGlobalController`, which loads
`MainRobot: Kinova6DOF` from `mc_rtc.yaml`) and the FSM state (via
`ctl.robot()`) actually operate on — the URDF's kinematic/dynamic parameters
are the ground truth behind every inverse-dynamics and mass-matrix
computation in §4–5. The tool payload (gripper) is *not* in this URDF at all;
it's approximated as the point mass described in §4.2 step 4, entirely
inside the bridge, rather than by editing the URDF.

---

## 7. Notable failure modes fixed along the way

Reading these alongside the sections above should make clear *why* each
mechanism exists, not just what it does.

### 7.1 Segfault on first control tick — uninitialized Jacobian output buffer

`rbd::Jacobian::fullJacobian(mb, jac, out)` writes into `out` but does not
resize it. The payload Jacobian's output matrix must be pre-allocated once at
init:
```cpp
payload_jac_full_ = Eigen::MatrixXd::Zero(6, gc_->robot().mb().nrDof());
```
Omitting this crashes on the very first tick that computes payload
compensation.

### 7.2 Arm rigid, does not move when pushed

Caused by anchoring the `PostureTask` target to the measured (encoder) pose
instead of the control robot's own state — see §5.1. Fixed by anchoring to
`ctl.robot().mbc().q` and driving both `target` and `refVel` together.

### 7.3 Runaway QP / segfault from loop-rate mismatch

Caused by running `gc_->run()` on a fixed-period ROS timer whose period did
not match the controller's configured `Timestep`. Fixed by reading
`gc_->timestep()` once at init and building the `rclcpp::TimerBase` from that
exact value (§4.3).

### 7.4 6–7 Hz oscillation at full vertical extension

Caused by feeding a torque-derived velocity command straight into the
solver with a configuration-dependent effective loop gain (§5.3). Fixed by
`velocityTau` (temporal low-pass on the command) and `inertiaScaling`
(spatial gain compensation via `H_ii(stance)/H_ii(q)`) together — neither
alone was as effective as the combination measured on logged data.

### 7.5 `SAFETY GATE` tripping on continuous joints past ±180°

Caused by comparing the model's unwrapped `q` to the driver's `[-pi, pi]`-
wrapped encoder reading with plain subtraction. Fixed by routing every
model-vs-encoder comparison through `angleDiff` (`std::remainder`-based),
and by expressing the published command as "encoder + clamped delta" so it
is always in the encoder's own wrapping (§4.4).

### 7.6 Wrist joint (joint_4/joint_6) artificially limited

Caused by the mc_kinova 6-DOF patch inheriting 7-DOF joint-limit overrides
for joints that are unbounded on the 6-DOF arm. Fixed in the mc_kinova patch
itself — see §6 and `patches/mc_kinova/README.md`.

---

## 8. Replicating this on a new arm/setup: a build order

If reproducing this system from scratch (new robot, new mc_rtc install,
etc.), the dependency order is:

1. **Robot description + `mc_rbdyn::RobotModule`.** Get a URDF that matches
   the physical arm (joint types — especially which joints are
   `continuous` — and limits must be right; see §6), and a
   `RobotModule` that loads it under a name you'll reference as `MainRobot`.
2. **`mc_rtc.yaml`.** `MainRobot`, `Enabled` (controller name), `Timestep`,
   `Plugins: [ROS]`, `ControllerModulePaths`/`StatesLibraries`/`StatesFiles`
   pointing at your build's install locations. Confirm which config file
   path your `mc_rtc` build actually reads
   (`~/.config/mc_rtc/mc_rtc.yaml` here) — a copy sitting only in-repo and
   never loaded was a real time sink during development of this package.
3. **A minimal FSM controller** (`KinovaHandGuiding.cpp`): just wraps
   `fsm::Controller`, nothing custom needed at this layer.
4. **FSM states**: a stiff `HoldPosition`, the admittance `HandGuide` state
   (§5), and the release-pose re-seed state (§3's `UpdateHoldTarget`). Get
   `HoldPosition` ⇄ `HandGuide` working and GUI-driven before adding
   anything else.
5. **The ROS 2 bridge node**, built incrementally:
   a. Seed `MCGlobalController` from the first `/joint_states` message and
      call `gc_->init(...)`; get `gc_->run()` being called at the right
      rate (§4.3) with no downstream publishing yet — verify in `dry_run`.
   b. Add the inverse-dynamics external-torque estimator (§4.2), publish it
      for inspection (`/admittance/tau_ext*`) before wiring it into the FSM.
   c. Wire the estimate into the datastore key the FSM state reads.
   d. Add command publishing with the angle-wrap-safe clamp and both safety
      gates (§4.4) *before* ever setting `dry_run: false`.
6. **Downstream `ros2_control` stack**: standard `joint_state_broadcaster` +
   `JointTrajectoryController` (position command interface,
   `open_loop_control: true` recommended so velocity hints are used for
   interpolation) — this package assumes it, doesn't provide it.
7. Tune gains on hardware using the GUI (§5.5), starting conservative
   (low `admittance`, `inertiaScaling: true`), then adjust per
   `CHANGES_2026-09-24.md`'s tables.

---

## 9. Where to look for what

| Question | File |
|---|---|
| How is external torque estimated? | `src/kortex_mc_rtc_bridge_admittance.cpp`, `controlLoop()` |
| How is a command turned into a safe ROS trajectory point? | same file, §4.4 (`angleDiff`, gates) |
| What's the actual admittance/velocity law? | `src/states/HandGuideState.cpp`, `run()` |
| What happens on FSM transitions? | `etc/KinovaHandGuiding.yaml` (`transitions`), `src/states/*` |
| What are the default gains and how to retune them? | `etc/KinovaHandGuiding.yaml` + `CHANGES_2026-09-24.md` |
| Which robot model/joint limits are used? | `patches/mc_kinova/README.md`, `patches/mc_kinova/mc_kinova_6dof.patch` |
| What ROS params does the bridge take, and their defaults? | `launch/admittance_real.launch.py`, constructor of `KortexMcRtcBridge` |
| Downstream `ros2_control` configuration | `config/ros2_controllers.yaml` |

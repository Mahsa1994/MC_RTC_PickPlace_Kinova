# =============================================================================
#  TUNING_GUIDE.md — KinovaGravityComp Admittance Parameters
# =============================================================================

## Overview of the Admittance Law

For each Cartesian axis *i*:

```
v_ref(i) += admittance(i) * (f_measured(i) - f_desired(i))
x_ref    += v_ref * dt
```

`x_ref` is the target pose fed to the QP.  `f_desired = 0` everywhere (free
float).  The Kortex API's estimated wrench already subtracts the arm's own
weight, so `f_measured ≈ 0` when no one is touching the arm.

---

## Parameter Reference

| Parameter | Location in YAML | Units | Effect |
|-----------|-----------------|-------|--------|
| `admittance` | `tasks.GravComp.admittance` | m/s/N or rad/s/N·m | Scales operator force to velocity command. Higher = arm feels lighter / more responsive. |
| `stiffness` | `tasks.GravComp.stiffness` | — | Near-zero removes position spring. Never set > 1 in hand-guiding mode. |
| `damping` | `tasks.GravComp.damping` | — | Viscous braking. Higher = more sluggish but more stable at release. |
| `maxVel.linear` | `tasks.GravComp.maxVel` | m/s | Hard cap on end-effector translational speed. |
| `maxVel.angular` | `tasks.GravComp.maxVel` | rad/s | Hard cap on end-effector rotational speed. |
| `weight` (GravComp) | `tasks.GravComp.weight` | — | QP weight vs other tasks. Must be higher than NullspacePosture weight. |
| `stiffness` (HoldPosture) | `tasks.HoldPosture.stiffness` | — | How stiffly the arm resists gravity in HoldPosition mode. |

---

## Tuning Procedure

### Step 1 — Verify force sensor readings
Before any motion, enter `HandGuide` with `admittance = [0,0,0,0,0,0]` (all
zeros).  Open the mc_rtc GUI → Log → `EndEffector_wrench`.  The readings should
be near zero at rest.  If you see a constant offset ≥ 2 N, the Kortex gravity
compensation calibration may need to be re-run.

### Step 2 — Start with translational admittance only
Set:
```yaml
admittance: [0.0003, 0.0003, 0.0003,  0.0, 0.0, 0.0]
damping:    [50.0,   50.0,   50.0,    50.0, 50.0, 50.0]
```
Push the end-effector by hand.  The arm should move slowly.

### Step 3 — Increase translational admittance until responsive
Typical range: `0.0003` (stiff feel) to `0.001` (free float).
Values > `0.002` may cause instability with high-inertia payloads.

### Step 4 — Enable rotational admittance
```yaml
admittance: [0.0005, 0.0005, 0.0005,  0.0004, 0.0004, 0.0004]
```
Rotational admittance is often set slightly lower than translational because
the arm's rotational inertia is lower and oscillations onset sooner.

### Step 5 — Tune damping for clean stop behaviour
Push and release the arm.  It should coast briefly then stop with no
oscillation.  If it oscillates → increase damping.  If it stops too
abruptly → decrease damping.

---

## Safety Considerations

| Risk | Mitigation |
|------|-----------|
| Arm accelerates unexpectedly | `maxVel.linear ≤ 0.15 m/s` and `maxVel.angular ≤ 0.30 rad/s` in YAML |
| Arm drifts under gravity when released | `UpdateHoldTarget` state re-seeds PostureTask from encoders |
| Operator pushes arm into self-collision | `type: collision` constraint in `constraints:` block |
| Kortex wrench offset causes slow drift | Re-run `kortex_calibration` tool; verify offset < 2 N |
| Joint limit approach feels hard | Tune `damper` parameters in `kinematics` constraint |

---

## Frequently Adjusted Values

**Arm feels sticky / hard to move:**
```yaml
admittance: [0.001, 0.001, 0.001, 0.0008, 0.0008, 0.0008]
damping:    [30.0,  30.0,  30.0,  30.0,   30.0,   30.0  ]
```

**Arm oscillates after release:**
```yaml
damping: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0]
```

**Arm drifts slowly when untouched (wrench offset):**
Add a deadband in a custom C++ state, or recalibrate Kortex gravity
compensation:
```bash
ros2 run mc_kortex kortex_gravity_calibration --robot-ip 192.168.1.10
```

**Hold stiffness insufficient (arm sags under gravity in HoldPosition):**
```yaml
stiffness: 15.0   # in HoldPosture task
weight:    200.0
```

---

## Live Parameter Tuning (no recompile)

In mc_rtc_rviz_panel or rqt_mc_rtc_gui:
- `Tasks → GravComp → admittance` — drag sliders per axis
- `Tasks → GravComp → damping`    — drag sliders per axis
- `Tasks → GravComp → maxVel`     — adjust safety limits

Changes take effect immediately in the running controller and can be
persisted back to YAML via `File → Save configuration`.

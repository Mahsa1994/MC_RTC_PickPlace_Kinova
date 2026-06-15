# KinovaGravityComp — mc_rtc Hand-Guiding Controller

Gravity-compensation / zero-g hand-guiding for the **Kinova Gen3 6-DOF** arm
using the **mc_rtc FSM + AdmittanceTask**.  No external F/T sensor required;
the Kortex API's built-in estimated wrench is used directly.

---

## File Structure

```
kinova_gravity_comp/
├── CMakeLists.txt                  Build system
├── TUNING_GUIDE.md                 Parameter reference and tuning procedure
├── etc/
│   ├── KinovaGravityComp.yaml      ← Main FSM controller config (edit gains here)
│   └── mc_rtc.yaml                 ← System-level config (robot IP, etc.)
└── src/
    └── states/
        ├── UpdateHoldTarget.h      Optional C++ state: re-seeds PostureTask
        └── UpdateHoldTarget.cpp    from encoders on release
```

---

## Architecture

### Why AdmittanceTask, not ImpedanceTask?

| Property | AdmittanceTask | ImpedanceTask |
|----------|---------------|---------------|
| Force → motion mapping | Integrates velocity from force error | Has a position-spring term |
| Behaviour when released | **Freezes at release pose** ✓ | Springs back to reference pose ✗ |
| Use case | Hand-guiding, gravity comp | Compliant trajectory following |

The admittance law `ẋ += K_adm × (f_meas − f_des)` with `f_des = 0` means:
- While the operator pushes: `f_meas ≠ 0` → arm moves.
- When released: `f_meas → 0` → velocity command → 0 → arm freezes.

No spring pulls the arm back.

### Why no external F/T sensor?

The Kinova Gen3 estimates the end-effector wrench from joint torque sensors
and a dynamics model, with gravity compensation applied internally.  The
`mc_kortex` driver publishes this as a virtual force sensor named
`EndEffectorForceSensor`.  Performance is slightly noisier than a dedicated
ATI or Rokubi sensor, but sufficient for hand-guiding at low speeds.

### FSM State Machine

```
┌─────────────────────────────────────────────────────────────────┐
│                                                                 │
│  Init ──(Auto, 1 cycle)──► HoldPosition                        │
│                                  │  ▲                          │
│                    GUI "Start"   │  │  GUI "Stop"              │
│                                  ▼  │                          │
│                             HandGuide                          │
│                          (AdmittanceTask,                      │
│                           all 6 DoF floating)                  │
└─────────────────────────────────────────────────────────────────┘
```

`HoldPosition` uses a `PostureTask` with stiffness 10 to resist gravity.
When transitioning back from `HandGuide`, the optional `UpdateHoldTarget`
C++ state re-seeds the PostureTask from current encoder values so the arm
freezes exactly where it was released — not where it was at controller start.

---

## Build & Install

```bash
# Prerequisites: mc_rtc, mc_kortex, ROS 2 (optional for visualization)
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
sudo make install
```

---

## Running

1. Copy `etc/mc_rtc.yaml` to `~/.config/mc_rtc/mc_rtc.yaml`
2. Edit the Kortex `ip_address` to match your robot.
3. Launch the mc_kortex interface:
   ```bash
   ros2 run mc_kortex mc_kortex_driver
   ```
4. Open the GUI:
   ```bash
   ros2 launch mc_rtc_rviz_panel display.launch.py
   ```
5. In the GUI → `Control` panel → click **▶ Start Hand-Guiding**.
6. Grab and move the arm freely.
7. Click **⏹ Stop Hand-Guiding (Hold Position)** to freeze the arm.

---

## Key Tuning Parameters (in `etc/KinovaGravityComp.yaml`)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `admittance` | `[0.0005 × 6]` | Higher = lighter feel |
| `damping` | `[50.0 × 6]` | Higher = firmer stop |
| `maxVel.linear` | `0.15 m/s` | Safety speed cap |
| `maxVel.angular` | `0.30 rad/s` | Safety speed cap |
| `HoldPosture.stiffness` | `10.0` | Stiffer = resists gravity better |

See `TUNING_GUIDE.md` for a full step-by-step procedure.

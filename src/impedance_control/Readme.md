# Kinova-mc_rtc Hand-Guiding Controller


---

## File Structure

```
```

---

## Architecture

### Why ImpedanceTask


### Why no external F/T sensor?

The Kinova Gen3 estimates the end-effector wrench from joint torque sensors
and a dynamics model, with gravity compensation applied internally.  This
repo's own bridge (`kortex_mc_rtc_bridge_impedance.cpp`) does that estimation
(joint torques + Jacobian transpose, damped-least-squares regularized) and
injects it under the sensor name `EEForceSensor` - that's the exact name
`ImpedanceHoldState::start()` checks for. Keep them in sync if you ever
rename it. Performance is slightly noisier than a dedicated ATI or Rokubi
sensor, but sufficient for hand-guiding at low speeds.

### FSM State Machine

```
```

---

## Build & Install

# Prerequisites: mc_rtc, mc_kortex, ROS 2 (optional for visualization)

---

## Running

---

## Key Tuning Parameters (in `etc/KinovaGravityComp.yaml`)

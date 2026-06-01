# Gripper Action Client (ROS 2)

This ROS 2 node provides a command-line interface to control the Robotiq 2F-85 gripper attached to a Kinova Gen3 robot via the `/robotiq_gripper_controller/gripper_cmd` action interface. It also subscribes to `/joint_states` to report real-time feedback of the actual gripper position.

---

## ✅ Features

- Sends goal commands to open or close the Robotiq gripper.
- Receives feedback from `/joint_states` to verify the gripper’s actual joint position.
- Synchronous CLI interface (blocking) for accurate operation and user feedback.

---

## 📦 Dependencies

- ROS 2 (tested on **Jazzy**)
- `control_msgs` (for `GripperCommand` action)
- `sensor_msgs` (for `JointState` message)
- A working Robotiq 2F-85 gripper controller exposing:
  - `/robotiq_gripper_controller/gripper_cmd` (action server)
  - `/joint_states` (topic with gripper joint position)

---

## 🚀 How to Run

1. Make sure the robot and gripper simulation or hardware is running.
2. Build and source your workspace:
   ```bash
   colcon build --packages-select gripper_control
   source install/setup.bash


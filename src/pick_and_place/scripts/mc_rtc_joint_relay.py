#!/usr/bin/env python3
import time, sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

MODE = sys.argv[1] if len(sys.argv) > 1 else 'sim'

if MODE == 'real':
    RATE_HZ  = 10.0
    DEADBAND = 0.002
    TRAJ_SEC = 0.2
    TOPIC    = '/joint_trajectory_controller/joint_trajectory'
else:
    RATE_HZ  = 200.0
    DEADBAND = 0.0001
    TRAJ_SEC = 0.005
    TOPIC    = '/kinova_joint_controller/joint_trajectory'

class McRtcJointRelay(Node):
    def __init__(self):
        super().__init__('mc_rtc_joint_relay')
        self._last_t   = 0.0
        self._last_pos = None
        self._interval = 1.0 / RATE_HZ
        self.pub = self.create_publisher(JointTrajectory, TOPIC, 10)
        self.sub = self.create_subscription(
            JointState, '/control/kinova_6dof/joint_states',
            self.callback, 10)
        self.get_logger().info(
            f'Relay [{MODE}] → {TOPIC} @ {RATE_HZ}Hz traj={TRAJ_SEC}s')

    def callback(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return
        pos = [msg.position[idx[j]] for j in JOINTS]
        now = time.monotonic()
        if now - self._last_t < self._interval:
            return
        if self._last_pos and max(
                abs(pos[i]-self._last_pos[i]) for i in range(6)) < DEADBAND:
            return
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions  = pos
        pt.velocities = [0.0]*6
        s  = int(TRAJ_SEC)
        ns = int((TRAJ_SEC - s)*1e9)
        pt.time_from_start = Duration(sec=s, nanosec=ns)
        traj.points = [pt]
        self.pub.publish(traj)
        self._last_t   = now
        self._last_pos = pos

def main():
    rclpy.init()
    rclpy.spin(McRtcJointRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

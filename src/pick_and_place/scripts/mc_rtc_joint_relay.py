#!/usr/bin/env python3
"""
Relay node: converts mc_rtc's joint state output to JointTrajectoryController commands.

mc_rtc publishes its desired joint positions on:
  /control/kinova_6dof/joint_states  (sensor_msgs/JointState)

JointTrajectoryController accepts commands on:
  /kinova_joint_controller/joint_trajectory  (trajectory_msgs/JointTrajectory)

This node subscribes to the former and publishes to the latter at every tick.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

class McRtcJointRelay(Node):
    def __init__(self):
        super().__init__('mc_rtc_joint_relay')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/kinova_joint_controller/joint_trajectory',
            #'/joint_trajectory_controller/joint_trajectory',
            10)
        self.sub = self.create_subscription(
            JointState,
            '/control/kinova_6dof/joint_states',
            self.callback,
            10)
        self.get_logger().info('mc_rtc joint relay started')

    def callback(self, msg: JointState):
        # Build index map from incoming message
        idx = {name: i for i, name in enumerate(msg.name)}
        if not all(j in idx for j in JOINTS):
            return  # not all joints present yet

        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [msg.position[idx[j]] for j in JOINTS]
        pt.velocities = [0.0] * len(JOINTS)
        # time_from_start = 0 means "execute immediately"
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 5ms
        traj.points = [pt]
        self.pub.publish(traj)

def main():
    rclpy.init()
    node = McRtcJointRelay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

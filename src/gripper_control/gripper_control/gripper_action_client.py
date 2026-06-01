#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState

class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_action_client')
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.gripper_position = None
        self.joint_found = False

        # Wait for initial joint state data
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=1.0)
            if self.joint_found:
                break

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name == 'robotiq_85_left_knuckle_joint':
                self.gripper_position = msg.position[i]
                self.joint_found = True
                self.get_logger().info(f'[INFO] current position: {self.gripper_position:.2f}')
                break

    def send_goal_sync(self, position):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('[INFO] Action server not available!')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = max(0.0, min(0.8, position))

        self.get_logger().info(f'[INFO] Sending goal - Position: {position:.2f}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('[INFO] Goal rejected by server!')
            return

        self.get_logger().info('[INFO] Goal accepted by server!')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        # Force-update joint state feedback after result
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.gripper_position is not None:
                break

        if self.gripper_position is not None:
            self.get_logger().info(f'[INFO] Goal completed - current position: {self.gripper_position:.2f}')
        else:
            self.get_logger().info('[INFO] Goal completed - current position: unavailable')

def main():
    rclpy.init()
    client = GripperActionClient()
    print("Enter position (0.0 = open, 0.8 = close), or '-1' to quit")
    print("Format: <position> (e.g., '0.5')\n")

    while True:
        try:
            user_input = input("Input (position): ")
            if not user_input:
                continue

            values = user_input.split()
            if len(values) == 1 and values[0] == '-1':
                client.get_logger().info('[INFO] Program terminated by user')
                break

            if len(values) != 1:
                print("[ERROR] Enter a single position (e.g., '0.8') or '-1'")
                continue

            position = float(values[0])
            if not (0.0 <= position <= 0.8):
                print(f"[ERROR] Position must be 0.0 to 0.8 (got {position})")
                continue

            client.send_goal_sync(position)

        except ValueError:
            print("[ERROR] Enter a valid number (e.g., '0.8') or '-1'")
        print("--------------------------")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


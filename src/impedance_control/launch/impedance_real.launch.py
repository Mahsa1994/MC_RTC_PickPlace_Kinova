from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Prereq: kortex_control.launch.py already running (driver + controller_manager).
    # This launch: spawn broadcasters/controllers, then the mc_rtc bridge.

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen')

    spawn_jtc = TimerAction(period=2.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager-timeout', '30'],
        output='screen')])

    spawn_gripper = TimerAction(period=3.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager-timeout', '30'],
        output='screen')])

    bridge = TimerAction(period=5.0, actions=[Node(
        package='impedance_control',
        executable='kortex_mc_rtc_bridge_impedance',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'dry_run': False,           # FLIP TO False ONLY AFTER SIGN CHECK
            'loop_dt': 0.001,
            'publish_decimation': 10,
            'delta_max': 0.01,         # tight for first trials
            'torque_sign': -1.0,        # flip to -1.0 if push test inverted
            'deadband_force': 6.0,
            'deadband_moment': 1.5,
        }])])

    return LaunchDescription([spawn_jsb, spawn_jtc, spawn_gripper, bridge])

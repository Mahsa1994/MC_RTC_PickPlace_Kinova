from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.10')

    # ── 1. Kortex driver (real robot) ─────────────────────────────────────
    kortex = Node(
        package='kortex_bringup',
        executable='kortex_control.launch.py',  
        output='screen'
    )

    # Better — use ExecuteProcess for launch files
    kortex = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'kortex_bringup', 'gen3.launch.py',
            'robot_ip:=192.168.1.10',
            'dof:=6',
            'use_fake_hardware:=false',
            'launch_rviz:=false',
            'use_internal_bus_gripper_comm:=false',
        ],
        output='screen'
    )

    # ── 2. mc_rtc_ticker ─────────────────────────────────────────────────
    mc_rtc = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['/home/vscode/workspace/install/bin/mc_rtc_ticker'],
                output='screen'
            )
        ]
    )

    # ── 3. Relay (real mode) ──────────────────────────────────────────────
    relay = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3',
                     '/home/vscode/workspace/src/pick_and_place/scripts/mc_rtc_joint_relay.py',
                     'real'],   # ← real mode: 10Hz, 200ms trajectory
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        kortex,
        mc_rtc,
        relay,
    ])

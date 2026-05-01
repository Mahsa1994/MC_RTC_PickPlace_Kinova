from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ── 1. Kortex driver (real robot) ─────────────────────────────────────
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

    # ── 2. The new Custom C++ Closed-Loop Bridge ──────────────────────────
    mc_rtc_bridge = TimerAction(
        period=8.0, # Wait 8 seconds for kortex driver to load
        actions=[
            Node(
                package='pick_and_place',
                executable='kortex_mc_rtc_bridge',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        kortex,
        mc_rtc_bridge,
    ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f kortex_mc_rtc_bridge 2>/dev/null || true; '
             'pkill -9 -f mc_rtc_ticker 2>/dev/null || true; '
             'fuser -k 4242/tcp 4343/tcp 2>/dev/null || true; '
             'rm -f /tmp/mc_rtc*.ipc /tmp/mc_rtc*.sock; '
             'sleep 1'
        ],
        output='screen'
    )

    # ── 1. Kortex driver (real robot) ─────────────────────────────────────
    kortex = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'kortex_bringup', 'gen3.launch.py',
            'robot_ip:=192.168.1.10',
            'dof:=6',
            'use_fake_hardware:=false',
            'launch_rviz:=false',
            'use_internal_bus_gripper_comm:=false',
#            'robot_controller:=joint_group_position_controller',
            'controllers_file:=' + os.path.join(get_package_share_directory('pick_and_place'), 'config', 'ros2_controllers.yaml'),
        ],
        output='screen'
    )

#    mc_rtc_bridge = TimerAction(
#        period=12.0,
#        actions=[
#            ExecuteProcess(
#                cmd=[
#                    'ros2', 'launch', 'mc_rtc_ros_control', 'control.launch.py',
#                    'publish_to:=/joint_trajectory_controller/joint_trajectory',
#                    'subscribe_to:=/joint_states',
#                ],
#                output='screen'
#            )
#        ]
#    )

    # ── 2. The new Custom C++ Closed-Loop Bridge ──────────────────────────
    mc_rtc_bridge = TimerAction(
        period=12.0, # Wait 8 seconds for kortex driver to load
        actions=[
            Node(
                package='pick_and_place',
                executable='kortex_mc_rtc_bridge',
                output='screen',
                parameters=[{'use_sim_time': False}]
            )
        ]
    )

    return LaunchDescription([
        cleanup,
        kortex,
        mc_rtc_bridge,
    ])

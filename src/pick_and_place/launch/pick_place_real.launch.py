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
    # Launched under chrt -f 80 (SCHED_FIFO) so ros2_control_node inherits
    # real-time priority on fork/exec - without this the 1kHz control loop
    # intermittently overruns its budget under SCHED_OTHER (verified on
    # real hardware: manually elevating a running ros2_control_node PID via
    # `chrt -f -p 80 <pid>` eliminated "Overrun detected!" warnings).
    kortex = ExecuteProcess(
        cmd=[
            'chrt', '-f', '80',
            'ros2', 'launch', 'kortex_bringup', 'gen3.launch.py',
            'robot_ip:=192.168.1.10',
            'dof:=6',
            'use_fake_hardware:=false',
            'launch_rviz:=false',
            'use_internal_bus_gripper_comm:=true',
            'gripper:=robotiq_2f_85',
            'gripper_joint_name:=robotiq_85_left_knuckle_joint',
            'robot_type:=gen3',
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

    # ── 2. Closed-Loop Bridge ──────────────────────────────────────────────
    # Uses impedance_control's wrench-aware bridge (not this package's own
    # kortex_mc_rtc_bridge, see the note at the top of that file) so
    # ComplianceCartesianMove has a real measuredWrench() to react to.
    mc_rtc_bridge = TimerAction(
        period=12.0, # Wait 8 seconds for kortex driver to load
        actions=[
            Node(
                package='impedance_control',
                executable='kortex_mc_rtc_bridge_impedance',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    # SAFETY DEFAULT - still True (2026-08-19). Root cause of
                    # the second E-stop found: delta_max (below) was clamped
                    # every ~10ms publish cycle with a fixed 20ms
                    # time_from_start, implying a SUSTAINED ~1 rad/s (~57
                    # deg/s) per-joint velocity whenever the QP target lagged
                    # real position by more than delta_max - true almost the
                    # entire time during real motion, completely overriding
                    # the intended slow/smooth state duration. Lowered 20x
                    # below. Verify in dry-run (check logged q_cmd/enc_q
                    # deltas look sane) before flipping this back to False.
                    'dry_run': True,
                    'torque_sign': -1.0,
                    'deadband_force': 1.0,
                    'deadband_moment': 1.5,
                    'delta_max': 0.0005,
                }]
            )
        ]
    )

    return LaunchDescription([
        cleanup,
        kortex,
        mc_rtc_bridge,
    ])

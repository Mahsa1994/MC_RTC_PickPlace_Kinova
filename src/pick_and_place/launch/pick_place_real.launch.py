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
                    # SAFETY DEFAULT - still True (2026-08-19). Three E-stops
                    # happened testing MoveHome, each initially chased as a
                    # symptom (pose error, then delta_max) before the real
                    # root cause was found: the bridge's internal QP-solved
                    # robot model (gc_->robot()) integrates open-loop and is
                    # NEVER resynced to the real encoder-observed arm. It can
                    # race arbitrarily far ahead of reality (confirmed:
                    # near-identical multi-radian divergence seen in both a
                    # dry-run test and a live test with delta_max already cut
                    # 20x) - delta_max only ever bounded the PER-CYCLE step,
                    # never the ACCUMULATED gap, so nothing stopped that
                    # divergence from growing and corrupting both the wrench
                    # compensation and the FSM's own convergence checks.
                    # Fixed in kortex_mc_rtc_bridge_impedance.cpp: the
                    # divergence is now computed every tick and used as a
                    # hard publish gate (model_real_gate below) - if the
                    # model has drifted past that threshold from the real
                    # arm, the bridge refuses to publish and logs loudly
                    # instead of continuing to command a fictional position.
                    # delta_max restored closer to its original value now
                    # that it's a smoothness/step-rate limiter, not the last
                    # line of defense. Verify in dry-run first (limited
                    # power - dry-run never publishes, so it can't exercise
                    # the gate's effect on real tracking, only confirm the
                    # bridge still runs and logs sanely), then re-confirm
                    # E-stop operator present before flipping this to False.
                    'dry_run': True,
                    'torque_sign': -1.0,
                    'deadband_force': 1.0,
                    'deadband_moment': 1.5,
                    'delta_max': 0.005,
                    'model_real_gate': 0.05,
                }]
            )
        ]
    )

    return LaunchDescription([
        cleanup,
        kortex,
        mc_rtc_bridge,
    ])

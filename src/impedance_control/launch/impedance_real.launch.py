from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Standalone impedance / push test: Kortex driver + controllers + the
    # wrench-aware mc_rtc bridge running KinovaImpedance. The arm holds a
    # pose; push it and it yields along the push, then returns on release.
    #
    # 2026-10-12: this file previously started ONLY the three spawners and
    # the bridge, with a comment saying the driver was a prerequisite
    # started elsewhere. Nothing started it, so every spawner died with
    # "Could not contact service /controller_manager/list_controllers" and
    # the bridge sat forever at "Waiting for first /joint_states...". The
    # driver is now launched here. The spawners are KEPT: gen3.launch.py
    # normally spawns the controllers from controllers_file itself, in
    # which case these simply no-op, but they cost nothing and cover the
    # case where its default controller name does not match this config.

    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f kortex_mc_rtc_bridge 2>/dev/null || true; '
             'pkill -9 -f mc_rtc_ticker 2>/dev/null || true; '
             'fuser -k 4242/tcp 4343/tcp 2>/dev/null || true; '
             'rm -f /tmp/mc_rtc*.ipc /tmp/mc_rtc*.sock; '
             'sleep 1'
        ],
        output='screen')

    # ── 1. Kortex driver ──────────────────────────────────────────────────
    # chrt -f 80 (SCHED_FIFO) so ros2_control_node inherits real-time
    # priority on fork/exec - without it the control loop intermittently
    # logs "Overrun detected!" (verified on real hardware).
    #
    # controllers_file MUST be this package's own: it sets
    # allow_nonzero_velocity_at_trajectory_end: true. The bridge streams
    # single-point trajectories whose last point has nonzero velocity for
    # any actively-compliant joint; without that flag the JTC rejects every
    # goal silently and the arm stays rigid no matter what the impedance
    # task computes.
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
            'controllers_file:=' + os.path.join(
                get_package_share_directory('impedance_control'),
                'config', 'ros2_controllers.yaml'),
        ],
        output='screen')

    # ── 2. Controllers (no-op if gen3.launch.py already spawned them) ─────
    spawn_jsb = TimerAction(period=6.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen')])

    spawn_jtc = TimerAction(period=8.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager-timeout', '30'],
        output='screen')])

    spawn_gripper = TimerAction(period=9.0, actions=[Node(
        package='controller_manager', executable='spawner',
        arguments=['robotiq_gripper_controller', '--controller-manager-timeout', '30'],
        output='screen')])

    # ── 3. Bridge + KinovaImpedance ───────────────────────────────────────
    bridge = TimerAction(period=15.0, actions=[Node(
        package='impedance_control',
        executable='kortex_mc_rtc_bridge_impedance',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            # LIVE - the arm moves as soon as this connects. Unchanged from
            # the committed value; there is no dry stage before it.
            'dry_run': False,
            'loop_dt': 0.001,
            'publish_decimation': 10,
            'delta_max': 0.01,
            'torque_sign': -1.0,
            # 6.0 -> 1.0 on 2026-10-12, from the logs of the 2026-08-05 run
            # where the push test demonstrably worked:
            #   mc-control-KinovaImpedance-pullDown.csv
            #     peak |F| during the pull      1.026 N
            #     smallest nonzero |F| logged   0.978 N  -> deadband was ~1.0
            #     peak impedance offset         8.92 mm
            #     tool movement around the push 11.4 mm (Z-dominant)
            #   mc-control-KinovaImpedance-untouched.csv
            #     all 46,966 samples exactly 0.000 N - nothing clears the
            #     deadband while stationary and untouched, so 1.0 N is not
            #     so low that noise leaks through.
            # At 6.0 the peak of a real push (1.03 N) never clears the
            # threshold, the linear channel stays dead, and only the moment
            # channel (2.68 Nm vs 1.5) gets through - rotation, not the
            # translation the push test is looking for.
            'deadband_force': 1.0,
            'deadband_moment': 1.5,
        }])])

    return LaunchDescription([
        cleanup,
        kortex,
        spawn_jsb,
        spawn_jtc,
        spawn_gripper,
        bridge,
    ])

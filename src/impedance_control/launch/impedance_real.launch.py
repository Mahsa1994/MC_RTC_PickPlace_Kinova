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
            # 6.0 -> 1.0 -> 3.0, both changes on 2026-10-12.
            #
            # 1.0 came from the 2026-08-05 pullDown log (peak |F| 1.026 N,
            # smallest nonzero 0.978 N). That turned out to be an unusually
            # GENTLE pull that barely cleared its own deadband, so it was a
            # bad calibration point.
            #
            # The live run on 2026-10-12 settled it. A firm pull-down gives:
            #     peak |F|          12.69 N
            #     peak deflection   120 mm   (F/K = 12.69/100 = 127 mm)
            # i.e. a real pull is ~12x what Aug 5 recorded. Meanwhile, AFTER
            # the pull the arm settles ~15 mm off its start pose, where the
            # gravity model is wrong by 1-3 N. At deadband 1.0 that residual
            # passed straight through and drove a sustained limit cycle:
            # measuredWrench_y flipped sign on CONSECUTIVE samples
            # (-2.48, +0.14, -0.82, +1.45, -0.80, +1.86, ...), the classic
            # Nyquist-frequency signature of loop gain > 1.
            #
            # It is not the impedance filter ringing - with M=3, K=100, D=60
            # critical damping is 2*sqrt(K*M) = 34.6, so zeta = 1.73, firmly
            # overdamped. It is the outer loop: wrench -> offset -> motion ->
            # efforts -> wrench.
            #
            # 3.0 sits in the gap between the 1-3 N residual and the 12+ N of
            # a real pull. Raise toward 4-5 if any chatter remains; lower
            # toward 2 if a deliberate gentle pull stops registering.
            'deadband_force': 3.0,
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

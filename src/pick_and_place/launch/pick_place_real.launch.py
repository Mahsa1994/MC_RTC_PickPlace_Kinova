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
                    # SAFETY DEFAULT - still True (2026-08-19, updated
                    # 2026-08-24). Three E-stops happened testing MoveHome,
                    # each initially chased as a symptom (pose error, then
                    # delta_max) before the real root cause was found: the
                    # bridge's internal QP-solved robot model (gc_->robot())
                    # integrates open-loop and is NEVER resynced to the real
                    # encoder-observed arm - it can race arbitrarily far
                    # ahead of reality. Fixed in
                    # kortex_mc_rtc_bridge_impedance.cpp: divergence is now
                    # computed every tick and used as a hard publish gate
                    # (model_real_gate below) - if the model has drifted
                    # past that threshold, the bridge refuses to publish and
                    # logs loudly instead of continuing to command a
                    # fictional position. MoveHome also switched to
                    # joint-space (2026-08-24) with a lowered stiffness, to
                    # route around a stall found in the Cartesian version -
                    # see README problems 12-13.
                    #
                    # delta_max cut further (0.005 -> 0.002) for the FIRST
                    # live test of the new joint-space MoveHome specifically
                    # (2026-08-24): this is a HARD cap on real robot speed
                    # (~0.5 rad/s -> ~0.2 rad/s, i.e. ~28.6 deg/s ->
                    # ~11.5 deg/s sustained max) independent of whatever
                    # MoveHome's new stiffness:0.05 does internally - that
                    # value is an untested estimate (dry-run can't validate
                    # real tracking pace at all), so this gives a second,
                    # unconditional layer of caution: even if the estimate
                    # is wrong and ctl.robot() moves faster than expected,
                    # the real arm still cannot exceed this rate, and the
                    # model_real_gate above will halt publishing well before
                    # any dangerous divergence accumulates either way. Loosen
                    # back toward 0.005 once this specific test is confirmed
                    # safe.
                    #
                    # LIVE TEST 2026-08-26 RESULT: MoveHome converged
                    # correctly, but MoveToPick did not (ComplianceCartesian-
                    # Move's target had no v_max-style speed bound, so real
                    # joint_5 chronically lagged the model - up to 0.04 rad,
                    # never closing - ending 0.245 m / 0.95 rad short of
                    # pick_pose after settle_timeout, correctly FORCED
                    # ADVANCE-tagged). ReturnHome then had to cover more
                    # distance than usual in its fixed 4.0s duration,
                    # requested a peak velocity past what delta_max lets the
                    # real arm track, tripped model_real_gate almost
                    # immediately, and stalled: the real arm froze safely
                    # (gate correctly refused to publish - confirmed no
                    # dangerous motion, E-stop was not needed), but nothing
                    # resyncs ctl.robot() mid-run, so it could never recover
                    # on its own. This is the SAME missing-speed-bound bug
                    # already found and fixed for MoveHome/JointMove on
                    # 2026-08-24 (see above) - just never migrated onto
                    # CartesianMove/ComplianceCartesianMove, which ReturnHome
                    # and MoveToPick use. FIXED 2026-08-26 in
                    # PickPlaceStates.cpp: both states now compute an
                    # effective_duration_ the same way JointMove does
                    # (duration becomes a MINIMUM, stretched so peak
                    # linear/angular velocity stays under new v_max_lin_/
                    # v_max_ang_ config, default 0.05 m/s / 0.05 rad/s).
                    #
                    # DRY-RUN VALIDATED 2026-08-26 (bypass: init=MoveToPick):
                    # ReturnHome's duration correctly auto-stretched
                    # 4.00s -> 9.03s (matches 1.875*ori_delta/v_max_ang
                    # exactly), no QP failure, model-vs-real deviation
                    # plateaued instead of climbing unbounded, MoveToPick
                    # FORCED ADVANCE-tagged and ReturnHome held instead of
                    # advancing - both per design. init reverted to MoveHome.
                    # E-stop operator reconfirmed physically present
                    # immediately before this live test, per protocol. Same
                    # watch-items as before: delta_max/model_real_gate still
                    # apply regardless of state; revert to True immediately
                    # on anything genuinely unexpected (fast/jerky/wrong-
                    # direction motion, or a gate-trip that doesn't clear).
                    'dry_run': False,
                    'torque_sign': -1.0,
                    'deadband_force': 1.0,
                    'deadband_moment': 1.5,
                    'delta_max': 0.002,
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

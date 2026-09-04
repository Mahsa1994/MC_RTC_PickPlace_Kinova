from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f kortex_mc_rtc_bridge 2>/dev/null || true; '
             'fuser -k 4242/tcp 4343/tcp 2>/dev/null || true; '
             'rm -f /tmp/mc_rtc*.ipc /tmp/mc_rtc*.sock; '
             'sleep 1'],
        output='screen'
    )

    mc_rtc_bridge = Node(
        package='admittance_control',
        executable='kortex_mc_rtc_bridge_admittance',
        output='screen',
        parameters=[{
            # UPDATED 2026-09-02: bridge now estimates its own wrench from
            # /joint_states' effort field (see kortex_mc_rtc_bridge_admittance.cpp
            # header comment) instead of a nonexistent /EEForceSensor ROS topic.
            # Values below are carried over from impedance_control's live-validated
            # tuning on this exact physical arm (same sensor, same sign convention,
            # same dynamics) - do not treat the C++ defaults as safe for real
            # hardware, they're generic/sim placeholders.
            #
            # SAFETY DEFAULT: dry_run stays True until this has been dry-run
            # validated on THIS package/build first - reconfirm E-stop presence
            # immediately before ever flipping this to False.
            'dry_run': True,
            'torque_sign': -1.0,
            'deadband_force': 1.0,
            'deadband_moment': 1.5,
            'delta_max': 0.002,
            'model_real_gate': 0.05,
        }]
    )

    return LaunchDescription([
        cleanup,
        mc_rtc_bridge,
    ])

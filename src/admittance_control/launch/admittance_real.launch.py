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
            'dry_run': True,
            'torque_sign': -1.0,
            'deadband_torque': 0.5,
            'qd_gate_low': 0.5,
            'qd_gate_high': 2.0,
            'delta_max': 0.02,
            'model_real_gate': 0.05,
            'publish_rate': 200.0,
            'payload_mass': 0.9,
            'payload_com': [0.0, 0.0, -0.12],
        }]
    )

    return LaunchDescription([
        cleanup,
        mc_rtc_bridge,
    ])

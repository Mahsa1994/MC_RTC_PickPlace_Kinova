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
        output='screen'
    )

    return LaunchDescription([
        cleanup,
        mc_rtc_bridge,
    ])

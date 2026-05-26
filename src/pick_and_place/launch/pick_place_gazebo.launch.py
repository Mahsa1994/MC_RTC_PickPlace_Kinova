from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

set_gz_plugin_path = SetEnvironmentVariable(
    name='GZ_SIM_SYSTEM_PLUGIN_PATH',
    value='/opt/ros/jazzy/lib'
)

set_gz_resource = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value='/home/vscode/workspace/devel/catkin_data_ws/install/share'
          ':/home/vscode/workspace/devel/mc_kinova'
)

def generate_launch_description():
    pkg        = get_package_share_directory('pick_and_place')
#    urdf_file  = os.path.join(pkg, 'urdf', 'kinova_6dof_sim.urdf')
    urdf_file = os.path.join( '/home/vscode/workspace/src/pick_and_place/urdf/kinova_6dof_sim.urdf')
    world_file = os.path.join(pkg, 'worlds', 'pick_place.world')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f kortex_mc_rtc_bridge 2>/dev/null || true; '
             'pkill -9 -f mc_rtc_ticker 2>/dev/null || true; '
             'fuser -k 4242/tcp 4343/tcp 2>/dev/null || true; '
             'rm -f /tmp/mc_rtc*.ipc /tmp/mc_rtc*.sock; '
             'sleep 1'],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot', '-topic', 'robot_description'],
        output='screen'
    )

    gz_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    '/world/pick_place_world/model/robot/joint_state'
                    '@sensor_msgs/msg/JointState'
                    '[gz.msgs.Model',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/joint_trajectory_controller/joint_trajectory'
                    '@trajectory_msgs/msg/JointTrajectory'
                    ']gz.msgs.JointTrajectory',
                ],
                remappings=[
                    ('/world/pick_place_world/model/robot/joint_state',
                     '/joint_states'),
                ],
                output='screen'
            )
        ]
    )

    # [FIX 1] Unpause defined as a function returning a fresh ExecuteProcess
    # each time it is referenced, preventing the "executed more than once"
    # crash. The previous version defined unpause as a module-level object
    # which asyncio attempted to execute twice, causing InvalidStateError
    # and bringing down the entire launch.
    def make_unpause():
        return ExecuteProcess(
            cmd=['gz', 'service',
                 '-s', '/world/pick_place_world/control',
                 '--reqtype', 'gz.msgs.WorldControl',
                 '--reptype', 'gz.msgs.Boolean',
                 '--req', 'pause: false',
                 '--timeout', '1000'],
            output='screen'
        )

    # t=6s: unpause so the gz controller manager's clock starts ticking.
    # Controller activation requires sim to be stepping — paused sim causes
    # the exact 5s timeout seen in every previous log.
    unpause = TimerAction(
        period=6.0,
        actions=[make_unpause()]
    )

    # [FIX 2] Spawners pushed to t=10s and t=11s.
    # Previous attempts at t=5s, t=6s, t=8s, t=9s all timed out.
    # The gz controller manager consistently takes 7-8s from world load
    # to being ready for activation. t=10s gives 4s margin after unpause.
    spawn_jsb = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager-timeout', '30'],
            output='screen'
        )]
    )

    spawn_jtc = TimerAction(
        period=11.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller',
                       '--controller-manager-timeout', '30'],
            output='screen'
        )]
    )

    # t=20s: controllers active by t=11s, bridge gets 9s of margin.
    mc_rtc_bridge = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='pick_and_place',
                executable='kortex_mc_rtc_bridge',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        cleanup,
        set_gz_plugin_path,
        set_gz_resource,
        gazebo,
        robot_state_pub,
        spawn_robot,
        gz_bridge,
        unpause,
        spawn_jsb,
        spawn_jtc,
        mc_rtc_bridge,
    ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
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
    # Reuse pick_and_place's URDF and world — same robot
    urdf_file  = '/home/vscode/workspace/src/admittance_control/urdf/kinova_6dof_sim.urdf'
    world_file = os.path.join(
        get_package_share_directory('pick_and_place'), 'worlds', 'pick_place.world'
    )

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

    # Admittance only needs joint_state + clock + FORCE SENSOR bridged
    # No joint_trajectory topic needed
    gz_bridge = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    # Joint states
                    '/world/pick_place_world/model/robot/joint_state'
                    '@sensor_msgs/msg/JointState'
                    '[gz.msgs.Model',
                    # Clock
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    # *** Force/torque sensor — critical for admittance ***
                    '/world/pick_place_world/model/robot/link/bracelet_link/sensor/EEForceSensor/forcetorque'
                    '@geometry_msgs/msg/WrenchStamped'
                    '[gz.msgs.Wrench',
#                    '/EEForceSensor@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
                ],
                remappings=[
                    ('/world/pick_place_world/model/robot/joint_state', '/joint_states'),
                    ('/world/pick_place_world/model/robot/link/bracelet_link/sensor/EEForceSensor/forcetorque',
                     '/EEForceSensor'),
                ],
                output='screen'
            )
        ]
    )

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

    unpause = TimerAction(period=6.0, actions=[make_unpause()])

    spawn_jsb = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
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


    # Reuse the same bridge binary from pick_and_place
    mc_rtc_bridge = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='admittance_control',       # ← same package, same binary
                executable='kortex_mc_rtc_bridge_admittance',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    shutdown_cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        '''
                        pkill -f "gz sim" 2>/dev/null || true
                        pkill -f ros_gz_bridge 2>/dev/null || true
                        pkill -f robot_state_publisher 2>/dev/null || true
                        pkill -f kortex_mc_rtc_bridge_admittance 2>/dev/null || true
                        pkill -f controller_manager 2>/dev/null || true
                        '''
                    ],
                    output='screen'
                )
            ]
        )
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
        # No spawn_jtc — admittance controller doesn't use joint_trajectory_controller
        mc_rtc_bridge,
        shutdown_cleanup,
    ])

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
    urdf_file  = '/home/vscode/workspace/src/pick_and_place/urdf/kinova_6dof_sim.urdf'
    world_file = os.path.join(pkg, 'worlds', 'pick_place.world')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── 1. Clean stale sockets/ports ─────────────────────────────────────
    clean = ExecuteProcess(
        cmd=['bash', '-c',
             'rm -f /tmp/mc_rtc*.ipc && '
             'fuser -k 4242/tcp 4343/tcp 2>/dev/null || true && '
             'echo "Cleanup done"'],
        output='screen'
    )

    # ── 2. Gazebo PAUSED (no -r flag) ─────────────────────────────────────
    # The arm stays frozen at its spawn pose until we explicitly unpause.
    # This prevents gravity from collapsing the unactuated arm.
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],   # no -r → starts paused
        output='screen'
    )

    # ── 3. Robot state publisher ──────────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ── 4. Spawn robot ────────────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'robot', '-topic', 'robot_description'],
        output='screen'
    )

    # ── 5. ros_gz_bridge ─────────────────────────────────────────────────
    gz_bridge = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                arguments=[
                    '/world/pick_place_world/model/robot/joint_state'
                    '@sensor_msgs/msg/JointState'
                    '[gz.msgs.Model',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                remappings=[
                    ('/world/pick_place_world/model/robot/joint_state',
                     '/joint_states'),
                ],
                output='screen'
            )
        ]
    )


    # ── Spawn ros2_control controllers ───────────────────────────────────
    spawn_jsb = TimerAction(
        period=4.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['joint_state_broadcaster'], output='screen')]
    )

    spawn_jtc = TimerAction(
        period=5.0,
        actions=[Node(package='controller_manager', executable='spawner',
                      arguments=['kinova_joint_controller'], output='screen')]
    )

    # ── 6. mc_rtc_ticker ─────────────────────────────────────────────────
    mc_rtc = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['/home/vscode/workspace/install/bin/mc_rtc_ticker'],
                output='screen'
            )
        ]
    )

    # ── 7. Unpause Gazebo AFTER mc_rtc is initialised ─────────────────────
    # mc_rtc needs ~2-3 seconds to load after the ticker starts (at t=5s).
    # We unpause at t=10s, by which time mc_rtc is running and sending
    # valid joint commands. The arm is actuated before gravity acts on it.
    unpause = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'service', '-s', '/world/pick_place_world/control',
                     '--reqtype', 'gz.msgs.WorldControl',
                     '--reptype', 'gz.msgs.Boolean',
                     '--req', 'pause: false',
                     '--timeout', '1000'],
                output='screen'
            )
        ]
    )
    return LaunchDescription([
        clean,
        set_gz_plugin_path,
        set_gz_resource,
        gazebo,
        robot_state_pub,
        spawn_robot,
        gz_bridge,
        spawn_jsb,
        spawn_jtc,
        mc_rtc,
        unpause,
    ])

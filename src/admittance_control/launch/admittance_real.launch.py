import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # ADDED 2026-09-03. Until now this launch file started only the bridge and
    # the Kortex driver had to be brought up by hand in another terminal. Every
    # run so far was started WITHOUT `gripper:=robotiq_2f_85`, and that single
    # missing argument is why there is no end effector: kortex_control.launch.py
    # gates robot_hand_controller_spawner on `gripper != ""`, and the xacro only
    # attaches the Robotiq 2F-85 (and its ros2_control joint) when it is set.
    # Symptom in the 21:51 run's logs: KortexMultiInterfaceHardware reports
    # "Gripper joint name is 'robotiq_85_left_knuckle_joint'" and "Gripper
    # initial position is '0.873365'" - the gripper is physically there on the
    # internal bus - but export_state_interfaces lists joint_1..joint_6 only, no
    # gripper spawner runs, and nothing downstream (/joint_states, TF, the
    # mc_rtc model) ever sees an end effector.
    # Owning the bringup here is what keeps that argument from being forgotten.
    # Pass launch_driver:=false to keep starting gen3.launch.py by hand.
    declare_launch_driver = DeclareLaunchArgument(
        'launch_driver', default_value='true',
        description='Start the Kortex driver too. Set false if gen3.launch.py '
                    'is already running in another terminal.')

    cleanup = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f kortex_mc_rtc_bridge 2>/dev/null || true; '
             'fuser -k 4242/tcp 4343/tcp 2>/dev/null || true; '
             'rm -f /tmp/mc_rtc*.ipc /tmp/mc_rtc*.sock; '
             'sleep 1'],
        output='screen'
    )

    # Mirrors pick_and_place/launch/pick_place_real.launch.py, which is the one
    # bringup in this workspace that has always had the gripper. controllers_file
    # points at THIS package's config, which defines robotiq_gripper_controller,
    # fault_controller and twist_controller so all four spawners find what they
    # expect.
    kortex = ExecuteProcess(
        cmd=[
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
                get_package_share_directory('admittance_control'),
                'config', 'ros2_controllers.yaml'),
        ],
        condition=IfCondition(LaunchConfiguration('launch_driver')),
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

    # Same 12 s the pick_and_place bringup uses: the bridge blocks on the first
    # /joint_states anyway, but starting it before the controller manager is up
    # just means it seeds mc_rtc from a half-configured hardware interface.
    delayed_bridge = TimerAction(period=12.0, actions=[mc_rtc_bridge])

    return LaunchDescription([
        declare_launch_driver,
        cleanup,
        kortex,
        delayed_bridge,
    ])

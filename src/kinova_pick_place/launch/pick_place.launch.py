#####
# launch/pick_place.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    kinova_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('kortex_driver'),
                'launch',
                'kortex_driver.launch.py'
            )
        ]),
        launch_arguments={
            'robot_name': 'gen3',
            'ip_address': '192.168.1.10',
        }.items()
    )

    mc_rtc_node = Node(
        package='mc_rtc_ticker',
        executable='mc_rtc_ticker',
        parameters=[{
            'conf': os.path.join(
                get_package_share_directory('kinova_pick_place'),
                'config',
                'PickPlaceFSM.yaml'
            )
        }],
        output='screen'
    )

    return LaunchDescription([kinova_driver, mc_rtc_node])


import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('crazyflie_simulation')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'crazyflie.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'crazyflie_apartment.wbt')
    )

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'crazyflie'},
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ]
    )

    slamtoolbox_node = Node(
        parameters=[
        {'odom_frame': 'odom'},
        {'map_frame': 'world'},
        {'base_frame': 'crazyflie'},
        {'scan_topic': '/scan'},
        {'use_scan_matching': False},
        {'max_laser_range': 3.5},
        {'resolution': 0.1},
        {'minimum_travel_distance': 0.01},
        {'minimum_travel_heading': 0.001},
        {'map_update_interval': 0.1},
        {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        slamtoolbox_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
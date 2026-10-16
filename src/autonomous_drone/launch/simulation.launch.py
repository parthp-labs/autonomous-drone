from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_share = get_package_share_directory(
        'autonomous_drone'
    )

    rviz_config = os.path.join(
        package_share,
        'config',
        'rviz_config.rviz'
    )

    lidar_simulator = Node(
        package='autonomous_drone',
        executable='lidar_simulator',
        name='lidar_simulator',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        lidar_simulator,
        rviz,
    ])

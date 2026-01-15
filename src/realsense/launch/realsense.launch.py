import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg = 'realsense'
    pkg_share = get_package_share_directory(pkg)


    camera_node = Node(
        package=pkg,
        executable='realsense',
        name='realsense',
        output='screen',
        parameters=[{"use_sim_time": False}]
    )

    image_vis_node = Node(
        package=pkg,
        executable='image_visualizer',
        name='image_visualizer',
        output='screen',
        parameters=[{"use_sim_time": False}]
    )

    points_vis_node = Node(
        package=pkg,
        executable='points_visualizer',
        name='points_visualizer',
        output='screen',
        parameters=[{"use_sim_time": False}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{"use_sim_time": False}],
        arguments=['-d', os.path.join(pkg_share, 'config', 'config.rviz')]
    )

    return LaunchDescription([
        # rviz_node,
        camera_node,
        image_vis_node,
        points_vis_node
    ])

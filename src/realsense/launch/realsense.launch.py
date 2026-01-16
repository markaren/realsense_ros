import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
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

    start_vis_after_camera = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=camera_node,
            on_start=[image_vis_node, points_vis_node]
        )
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

        start_vis_after_camera,
        camera_node,
    ])

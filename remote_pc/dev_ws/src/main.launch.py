import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    base_path = os.path.join(get_package_share_directory('system'), 'config')
    rviz_config_path=base_path+'/standard.rviz'
    return LaunchDescription([
        # Node(
        #     package='system',
        #     executable='read_data',
        #     name='read_node'
        # ),
        Node(
            package='system',
            executable='static_tf',
            name='tf_node'
        ),
        # Node(
        #     package='system',
        #     executable='visualize',
        #     name='visualization_node'
        # ),
        Node(
            package='system',
            executable='anchor',
            name='anchor_node'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', str(rviz_config_path)]
        ),
    ])

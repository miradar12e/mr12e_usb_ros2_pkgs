from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    base_path = os.path.realpath(get_package_share_directory('miradar_node'))

    rviz_path = base_path + "/resource/miradar.rviz"

    return LaunchDescription([
        Node(
            package="miradar_node",
            executable="miradar_node",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="miradar_node",
            executable="ppi_visualizer.py",
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d'+rviz_path]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0","0","0","0","0","0", "miradar", "miradar_scan"]
        ),
        Node(
            package="rqt_reconfigure",
            executable="rqt_reconfigure"
        )
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Pfad zum display.launch.py
    hand_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('hand_description'),
                'launch',
                'display.launch.py'
            )
        ])
    )
    
    # Joint State Publisher GUI für die Slider
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )
    
    return LaunchDescription([
        hand_description_launch,
        joint_state_publisher_gui
    ])
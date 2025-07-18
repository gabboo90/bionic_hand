from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Leap Motion Environment Variables
    leap_env = {
        'LD_LIBRARY_PATH': '/home/gabi/Desktop/bionic_hand/src/leap_node/lib/x64:/home/gabi/Desktop/ros2_ws/leap_extracted/usr/lib/x86_64-linux-gnu:/home/gabi/Desktop/ros2_ws/leap_extracted/usr/lib'
    }
    
    # Leap Motion Daemon
    leapd = ExecuteProcess(
        cmd=["sudo", "/home/gabi/Desktop/ros2_ws/leap_extracted/usr/sbin/leapd"],
        output="screen",
    )
    
    # Static Transform: leap_frame -> world
    leap_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="leap_to_world_transform",
        arguments=["0", "0", "0", "1.57", "0", "1.57", "world", "leap_frame"],
        output="screen"
    )
    
    # Leap Motion Sample Node
        # Leap Motion Sample Node
    leap_sample = Node(
        package="leap_node",
        executable="sample_node", 
        name="leap_sample",
        output="screen"
        # Kein additional_env - nutzt die globalen Environment-Variablen
    )
    
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
        leapd,
        leap_transform,
        leap_sample,
        hand_description_launch,
        joint_state_publisher_gui
    ])
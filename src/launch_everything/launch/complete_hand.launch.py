from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # ROS 2 Environment laden
    ros_env = os.environ.copy()
    ros_env['LD_LIBRARY_PATH'] = '/home/gabi/Desktop/bionic_hand/src/leap_node/lib/x64:/home/gabi/Desktop/bionic_hand/src/leap_node/LeapSDK/lib/x64:/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:' + ros_env.get('LD_LIBRARY_PATH', '')
    
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
    leap_sample = Node(
        package="leap_node",
        executable="sample_node", 
        name="leap_sample",
        output="screen",
        env=ros_env
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

    # Servo Mapper Node - wandelt joint_states in Servo-Kommandos um
    servo_mapper_node = Node(
        package="servo_mapper",
        executable="servo_mapper_node",
        name="servo_mapper_node",
        output="screen"
    )

    # Dynamixel Read Write Node - sendet Kommandos an den Servo
    read_write_node = Node(
        package="dynamixel_sdk_examples",
        executable="read_write_node",
        name="read_write_node",
        output="screen"
    )

    leap_joint_publisher = Node(
    package="leap_node",
    executable="dof_publisher_node",
    name="leap_joint_publisher",
    output="screen",
    env=ros_env
    )

    return LaunchDescription([
        leapd,
        leap_transform,
        leap_sample,
        hand_description_launch,
        # joint_state_publisher_gui,
        servo_mapper_node,
        leap_joint_publisher
    ])
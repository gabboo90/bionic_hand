#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory

class URDFPublisher(Node):
    def __init__(self):
        super().__init__('urdf_publisher')
        
        # URDF-Pfad finden
        package_dir = get_package_share_directory('hand_description')
        urdf_path = os.path.join(package_dir, 'urdf', 'hand.urdf')
        
        # URDF-Datei lesen
        with open(urdf_path, 'r') as f:
            urdf_content = f.read()
        
        # Parameter setzen
        self.declare_parameter('robot_description', urdf_content)
        
        self.get_logger().info('URDF loaded and parameter set!')

def main():
    rclpy.init()
    node = URDFPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

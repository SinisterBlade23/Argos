#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import numpy as np

class defaultPosition(Node):
    def __init__(self):
        super().__init__('default_position')


        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10)
        
        self.subscribe = self.create_subscription(
            String,
            'motion_cmd',
            self.default_position_publisher,
            10
        )
        
        self.default_position = [0.0, 0.6, -0.10, 0.0, -0.6, -0.10, 0.0, -0.6, 0.10, 0.0, -0.6, -0.10]
        
        

    def default_position_publisher(self,msg):
        if msg.data == 'stop':
            self.get_logger().info(f'recieved "{msg.data}"')
            joint_msg = Float64MultiArray()
            joint_msg.data = self.default_position
            self.publisher.publish(joint_msg)

def main(args = None):
    rclpy.init(args=args)
    node = defaultPosition()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == '__main__':
    main()
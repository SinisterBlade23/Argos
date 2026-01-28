#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class InterpolatedPublisher(Node):
    def __init__(self):
        super().__init__('interpolated_publisher')
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10
        )
        
        # FL three key positions
        self.positions = [
            [0.0, 1.2, 0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.8, 1.2,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.5, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        
        # Parameters
        self.frequency = 100  # Hz - publishing rate
        self.transition_time = 0.5 # seconds - time to move between positions
        self.hold_time = 0.001  # seconds - time to hold at each position
        
        # State variables
        self.current_pos_idx = 0
        self.next_pos_idx = 1
        self.t = 0.0  # time within current transition
        self.phase = 'transition'  # 'transition' or 'hold'
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        
        self.get_logger().info('Interpolated Publisher Started')
        self.get_logger().info(f'Publishing at {self.frequency} Hz')
        self.get_logger().info(f'Transition time: {self.transition_time}s, Hold time: {self.hold_time}s')
    
    def interpolate(self, pos1, pos2, t):
        """
        Smooth interpolation using cubic easing
        t: 0.0 to 1.0
        """
        # Cubic ease-in-out for smoother motion
        if t < 0.5:
            eased_t = 4 * t * t * t
        else:
            eased_t = 1 - pow(-2 * t + 2, 3) / 2
        
        # Linear interpolation with easing
        return [p1 + (p2 - p1) * eased_t for p1, p2 in zip(pos1, pos2)]
    
    def timer_callback(self):
        dt = 1.0 / self.frequency
        
        if self.phase == 'transition':
            # Calculate interpolation parameter (0.0 to 1.0)
            alpha = self.t / self.transition_time
            
            if alpha >= 1.0:
                # Transition complete, start hold phase
                self.phase = 'hold'
                self.t = 0.0
                current_position = self.positions[self.next_pos_idx]
            else:
                # Interpolate between positions
                current_position = self.interpolate(
                    self.positions[self.current_pos_idx],
                    self.positions[self.next_pos_idx],
                    alpha
                )
        
        else:  # hold phase
            current_position = self.positions[self.next_pos_idx]
            
            if self.t >= self.hold_time:
                # Hold complete, start next transition
                self.phase = 'transition'
                self.t = 0.0
                self.current_pos_idx = self.next_pos_idx
                self.next_pos_idx = (self.next_pos_idx + 1) % len(self.positions)
                
                self.get_logger().info(
                    f'Moving to position {self.next_pos_idx + 1}'
                )
        
        # Publish the position
        msg = Float64MultiArray()
        msg.data = current_position
        self.publisher.publish(msg)
        
        # Increment time
        self.t += dt

def main(args=None):
    rclpy.init(args=args)
    node = InterpolatedPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
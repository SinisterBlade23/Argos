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
        
        # Default starting position (activated only on launch)
        self.default_position = [0.0, 0.6, -0.10, 0.0, -0.6, -0.10, 0.0, -0.6, 0.10, 0.0, -0.6, -0.10]
        
        # FL three key positions
        self.positions = [
            [0.0, 0.2, 0.6, 0.0, -0.6, 0.10, 0.0, -0.6, 0.10, 0.0, -0.2, 0.6],  # FL+BR lift up
            [0.0, -0.3, -0.35, 0.0, -0.2, -0.6, 0.0, -0.2, 0.6, 0.0, 0.3, -0.35],  # FL+BR swing forward, FR+BL on ground
            [0.0, 0.6, -0.10, 0.0, 0.3, 0.35, 0.0, 0.3, -0.35, 0.0, -0.6, -0.10],  # FL+BR lower, FR+BL lift
            ]
        
        # Parameters
        self.frequency = 100  # Hz - publishing rate
        self.transition_time = 0.2  # seconds - time to move between positions
        self.hold_time = 0.03  # seconds - time to hold at each position
        self.startup_transition_time = 0.5  # seconds - time to move from default to first position
        
        # State variables
        self.is_startup = True  # Flag to indicate startup phase
        self.current_pos_idx = 0
        self.next_pos_idx = 0  # Start with first position after startup
        self.t = 0.0  # time within current transition
        self.phase = 'transition'  # 'transition' or 'hold'
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        
        self.get_logger().info('Interpolated Publisher Started')
        self.get_logger().info(f'Publishing at {self.frequency} Hz')
        self.get_logger().info(f'Transition time: {self.transition_time}s, Hold time: {self.hold_time}s')
        self.get_logger().info(f'Starting from default position, transitioning to position 1 in {self.startup_transition_time}s')
    
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
        
        # Handle startup phase (default position to first position)
        if self.is_startup:
            # Calculate interpolation parameter (0.0 to 1.0)
            alpha = self.t / self.startup_transition_time
            
            if alpha >= 1.0:
                # Startup transition complete
                self.is_startup = False
                self.phase = 'hold'
                self.t = 0.0
                self.current_pos_idx = 0
                self.next_pos_idx = 1
                current_position = self.positions[0]
                self.get_logger().info('Reached first position, starting normal operation')
            else:
                # Interpolate from default to first position
                current_position = self.interpolate(
                    self.default_position,
                    self.positions[0],
                    alpha
                )
        
        # Normal operation (after startup)
        elif self.phase == 'transition':
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
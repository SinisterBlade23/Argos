#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
import numpy as np

class InterpolatedPublisher(Node):
    def __init__(self):

        super().__init__('interpolated_publisher')


        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10
        )

        self.cmd_sub = self.create_subscription(
            String,
            'motion_cmd',
            self.cmd_callback,
            10            
        )
        
        # Default starting position (activated only on launch)
        self.default_position = [0.0, 0.8, -0.10, 0.0, -0.8, -0.10, 0.0, -0.6, 0.10, 0.0, -0.6, -0.10]
        
  
        self.positions = [
            [0.0, 0.8, 0.20, 0.0, -0.8, 0.0, 0.0, 0.2, 0.6, 0.0, -0.60, -0.10],     # FL+BR lift up
            [0.0, 0.8, 0.00, 0.0, -0.8, 0.35, 0.0, -0.3, -0.35, 0.0, -0.2, -0.6],  # FL+BR swing backward
            [0.0, 0.8, 0.35, 0.0, -0.8, 0.20, 0.0, -0.6, 0.10, 0.0, -0.3, 0.35],   # FL+BR lower, FR+BL lift
            ]
        
        # Parameters
        self.frequency = 100  # Hz - publishing rate
        self.transition_time = 0.2  #  time to move between positions
        self.hold_time = 0.03  # time to hold at each position
        self.startup_transition_time = 0.5  # time to move from default to first position
        
        
        self.is_startup = True  # Flag to indicate startup phase
        self.current_pos_idx = 0
        self.next_pos_idx = 0  # Start with first position after startup
        self.t = 0.0  # time within current transition
        self.phase = 'transition'  # 'transition' or 'hold'
        
        self.startup_done = False 
        self.current_cmd = 'idle'

        # Create timer
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        
        self.get_logger().info('Interpolated Publisher Started')
        self.get_logger().info(f'Publishing at {self.frequency} Hz')
        self.get_logger().info(f'Transition time: {self.transition_time}s, Hold time: {self.hold_time}s')
        self.get_logger().info(f'Starting from default position, transitioning to position 1 in {self.startup_transition_time}s')
    
    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'backward':
            if self.current_cmd != 'backward':
                self.current_cmd = 'backward'
                if self.startup_done:
                    self.phase = 'transition'
                    self.t = 0.0
                self.get_logger().info('walk backward')
        else:
            
            if self.current_cmd != 'idle':
                self.current_cmd = 'idle'
                self.get_logger().info(f'Received "{cmd}", stopping')

    def interpolate(self, pos1, pos2, t):
        if t < 0.5:
            eased_t = 4 * t * t * t
        else:
            eased_t = 1 - pow(-2 * t + 2, 3) / 2
        
        
        return [p1 + (p2 - p1) * eased_t for p1, p2 in zip(pos1, pos2)]
    
    def timer_callback(self):
        dt = 1.0 / self.frequency
        
        
        if self.is_startup:
            alpha = self.t / self.startup_transition_time
            
            if alpha >= 1.0:
               
                self.is_startup = False
                self.startup_done = True
                self.phase = 'hold'
                self.t = 0.0
                self.current_pos_idx = 0
                self.next_pos_idx = 1
                current_position = self.positions[0]
                self.get_logger().info('Reached first position, waiting for command...')
            else:
                current_position = self.interpolate(
                    self.default_position,
                    self.positions[0],
                    alpha
                )
            
            
            msg = Float64MultiArray()
            msg.data = current_position
            self.publisher.publish(msg)
            self.t += dt
            return
        
       
        if self.current_cmd != 'backward':
            return
        
        if self.phase == 'transition':
            alpha = self.t / self.transition_time
            
            if alpha >= 1.0:                
                self.phase = 'hold'
                self.t = 0.0
                current_position = self.positions[self.next_pos_idx]
            else:
                current_position = self.interpolate(
                    self.positions[self.current_pos_idx],
                    self.positions[self.next_pos_idx],
                    alpha
                )
        
        else:  
            current_position = self.positions[self.next_pos_idx]
            
            if self.t >= self.hold_time:                
                self.phase = 'transition'
                self.t = 0.0
                self.current_pos_idx = self.next_pos_idx
                self.next_pos_idx = (self.next_pos_idx + 1) % len(self.positions)
                self.get_logger().  info(
                    f'Moving to position {self.next_pos_idx + 1}'
                )
        
        
        msg = Float64MultiArray()
        msg.data = current_position
        self.publisher.publish(msg)
        
        
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
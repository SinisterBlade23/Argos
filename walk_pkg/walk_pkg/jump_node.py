#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
import numpy as np


class JumpNode(Node):
    def __init__(self):
        super().__init__('jump_node')
        
       
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10
        )
        
        
        self.cmd_sub = self.create_subscription(
            String,
            '/motion_cmd',
            self.cmd_callback,
            10
        )
        
        
        self.crouch_position = [0.0, 1.3, 1.0, 0.0, -1.3, 1.0, 0.0, -1.3, -1.0, 0.0, -1.3, 1.0]
        self.extend_position = [0.0, 0.6, -0.10, 0.0, -0.6, -0.10, 0.0, -0.6, 0.10, 0.0, -0.6, -0.10]
        
        # Current command
        self.current_cmd = 'idle'
        
        
        self.jump_phase = 'idle'  
        self.t = 0.0
        self.jump_active = False
        
        # Timing parameters
        self.frequency = 100  # Hz
        self.crouch_time = 0.8  # Slow crouch - 0.8 seconds
        self.explode_time = 0.1 # explosion speed - 50ms
        self.retract_time = 0.05  # retraction time
        self.hold_time = 0.5  # Hold crouch after landing - 0.5 seconds
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        
        self.get_logger().info('Jump Node Started (Modular)')
        self.get_logger().info('Listening for "jump" command on /motion_cmd')
        self.get_logger().info('Will stop publishing if another command is received')
    
    def cmd_callback(self, msg):
        
        cmd = msg.data.strip().lower()
        
        if cmd == 'jump':
            if not self.jump_active:
                self.get_logger().info('JUMP command received!')
                self.current_cmd = 'jump'
                self.jump_active = True
                self.jump_phase = 'crouch'
                self.t = 0.0
            else:
                self.get_logger().info('Already jumping...')
        else:
            # Another command received - stop publishing
            if self.current_cmd == 'jump':
                self.get_logger().info(f'Received "{cmd}" - Jump node stopping publication')
                self.current_cmd = cmd
                self.jump_active = False
                self.jump_phase = 'idle'
    
    def interpolate_linear(self, pos1, pos2, t):
        return [p1 + (p2 - p1) * t for p1, p2 in zip(pos1, pos2)]
    
    def interpolate_cubic(self, pos1, pos2, t):        
        if t < 0.5:
            eased_t = 4 * t * t * t
        else:
            eased_t = 1 - pow(-2 * t + 2, 3) / 2
        
        return [p1 + (p2 - p1) * eased_t for p1, p2 in zip(pos1, pos2)]
    
    def timer_callback(self):
        dt = 1.0 / self.frequency
        
        
        if self.current_cmd != 'jump':
            return
        if self.jump_phase == 'idle':
            # just in case idk
            return
        
        elif self.jump_phase == 'crouch':
            
            alpha = self.t / self.crouch_time
            
            if alpha >= 1.0:
                
                self.jump_phase = 'explode'
                self.t = 0.0
                target_position = self.crouch_position
                self.get_logger().info('Phase: EXPLODE - Jumping NOW!')
            else:
                
                target_position = self.interpolate_cubic(
                    self.extend_position,
                    self.crouch_position,
                    alpha
                )
        
        elif self.jump_phase == 'explode':
            alpha = self.t / self.explode_time
            
            if alpha >= 1.0:
                self.jump_phase = 'retract'
                self.t = 0.0
                target_position = self.extend_position
                self.get_logger().info('Phase: RETRACT')
            else:                
                target_position = self.interpolate_linear(
                    self.crouch_position,
                    self.extend_position,
                    alpha
                )
        
        elif self.jump_phase == 'retract':
            
            alpha = self.t / self.retract_time
            
            if alpha >= 1.0:
                
                self.jump_phase = 'hold'
                self.t = 0.0
                target_position = self.crouch_position
                self.get_logger().info('Phase: HOLD ')
            else:                
                target_position = self.interpolate_linear(
                    self.extend_position,
                    self.crouch_position,
                    alpha
                )
        
        elif self.jump_phase == 'hold':
            # Hold crouch position
            target_position = self.crouch_position
            
            if self.t >= self.hold_time:
                
                self.jump_phase = 'idle'
                self.jump_active = False
                self.current_cmd = 'idle'
                self.t = 0.0
                self.get_logger().info('Jump sequence COMPLETE!.')
                return  # Stop publishing
        
        else:
            return
        
        msg = Float64MultiArray()
        msg.data = target_position
        self.publisher.publish(msg)
        
        
        self.t += dt


def main(args=None):
    rclpy.init(args=args)
    node = JumpNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
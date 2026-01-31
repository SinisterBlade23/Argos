import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardWASD(Node):
    def __init__(self):
        super().__init__('keyboard_wasd')
        self.publisher = self.create_publisher(String, 'motion_cmd', 10)
        self.get_logger().info("WASD control ready (w/s/a/d, x = stop, q = quit)")

        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            msg = String()

            if key == 'w':
                msg.data = 'forward'
            elif key == 's':
                msg.data = 'backward'
            elif key == 'a':
                msg.data = 'left'
            elif key == 'd':
                msg.data = 'right'
            elif key == 'x':
                msg.data = 'stop'
            elif key == 'q':
                break
            else:
                continue

            self.publisher.publish(msg)
            self.get_logger().info(f"Command: {msg.data}")

def main():
    rclpy.init()
    node = KeyboardWASD()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

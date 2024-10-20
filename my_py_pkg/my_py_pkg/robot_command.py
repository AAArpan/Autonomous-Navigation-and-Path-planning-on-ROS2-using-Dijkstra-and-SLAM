import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Keyboard Control Node has been started.')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run(self):
        try:
            while True:
                key = self.get_key()
                twist = Twist()
                if key == 'w':
                    twist.linear.x = 0.5
                    twist.angular.z = 0.0
                elif key == 's':
                    twist.linear.x = -0.5
                    twist.angular.z = 0.0
                elif key == 'a':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == 'd':
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
                elif key == '\x03': 
                    break
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                self.publisher_.publish(twist)
                self.get_logger().info(f'Publishing: linear_x={twist.linear.x}, angular_z={twist.angular.z}')

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().info('Node stopped cleanly.')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

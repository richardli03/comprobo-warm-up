import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios

class Teleop(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1
        # self.run_loop()
        self.timer = self.create_timer(timer_period, self.run_loop)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

    def run_loop(self):
        
        msg = Twist()
        key = self.get_key() # THIS WILL BLOCK

        if key == '\x03':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            raise KeyboardInterrupt("INTERRUPTED")

        if key == "a":
            msg.linear.x = 0.0
            msg.angular.z = 1.0
        elif key == "d":
            msg.linear.x = 0.0
            msg.angular.z = -1.0
        elif key == "w":
            msg.angular.z = 0.0
            msg.linear.x = 1.0
        elif key == "s":
            msg.angular.z = 0.0
            msg.linear.x = -1.0
        elif key == "q":
            msg.angular.z = 0.0
            msg.linear.x = 0.0

        self.publisher.publish(msg)
    
        
def main(args=None):
    rclpy.init(args=args) # init the node
    node = Teleop()
    rclpy.spin(node) # starts up the node
    rclpy.shutdown() # if it finishes, it'll shutdown
    
if __name__ == '__main__':
    main()

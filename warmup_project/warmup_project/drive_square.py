import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import sleep 
from math import pi
import tty
import select
import sys
import termios

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('square')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom = self.create_subscription(Odometry, 'odom', self.update_position, 10)
        timer_period = 1
        # self.run_loop()
        self.timer = self.create_timer(timer_period, self.run_loop)

    def move(self, lin, ang):
        """move

        :param lin: _description_
        :type lin: _type_
        :param ang: _description_
        :type ang: _type_
        """
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.publisher.publish(msg)
        
    def update_position(self, msg):
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.zpos = msg.pose.pose.position.z
        
        self.orientation = msg.pose.pose.orientation
        print(self.xpos, self.ypos, self.zpos, self.orientation)
        
    def stop(self):
        self.move(0.0,0.0)
        
    def turn(self, angle):
        """turn 

        :param angle: angle in degrees
        :type angle: _type_
        """
        angular_vel = 0.3
        time_to_sleep = (pi / angular_vel / 2) * (4) * (angle/360) # 90 degrees 
    
        self.move(0.0, angular_vel)
        sleep(time_to_sleep)
        self.move(0.0, 0.0)
        
    def run_loop(self):
        for _ in range(4):
            self.move(1.0, 0.0)
            sleep(1)
            self.stop()
            self.turn(90)
            
    


    
        
def main(args=None):
    rclpy.init(args=args) # init the node
    node = DriveSquareNode()
    rclpy.spin(node) # starts up the node
    rclpy.shutdown() # if it finishes, it'll shutdown
    
if __name__ == '__main__':
    main()

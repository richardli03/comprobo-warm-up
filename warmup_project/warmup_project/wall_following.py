import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pi
from enum import Enum, auto

class States(Enum):
    LEFT = auto()
    RIGHT = auto()
    STRAIGHT = auto()


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follow')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_msg = self.create_subscription(Odometry, 'odom', self.update_position, 10)
        self.scan_msg = self.create_subscription(LaserScan, 'scan', self.scan, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.state = None
        
        
    def move(self, lin = 0.0, ang = 0.0):
        """move

        :param lin: _description_
        :type lin: _type_
        :param ang: _description_
        :type ang: _type_
        """
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.publisher.publish(msg)
        
    def scan(self, msg):
        straight_ahead_dist = msg.ranges[0]
        theta1_dist = msg.ranges[45]
        theta2_dist = msg.ranges[45 + 90]
        # print(straight_aead_dist)
        print(f"THETA1 was {theta1_dist}, theta2_dist was {theta2_dist}")
        
        if straight_ahead_dist < 0.75:
            # CRASHING INTO A WALL! OH NO!
            self.turn(15) # turn 10 degrees to the left, should fix the issue
        
        if abs(theta1_dist - theta2_dist) < 0.5:
            self.move(0.5, 0.0)
            sleep(0.1)
            
        if theta1_dist > theta2_dist:
            # print("turning right")
            self.state = States.RIGHT

        elif theta2_dist > theta1_dist:
            # print("turning left")
            self.state = States.LEFT
            pass
        else:
            # print("moving straight")
            self.state = States.STRAIGHT
            pass
        
        
    def update_position(self, msg):
        """
        Set some attributes according to wheel encoder data.

        :param msg: _description_
        :type msg: _type_
        """
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.zpos = msg.pose.pose.position.z
        
        self.orientation = msg.pose.pose.orientation
        # print(self.xpos, self.ypos, self.zpos, self.orientation)
        
    def stop(self):
        """Writing a stop method to make code more readable
        """
        self.move(0.0,0.0)
        
    def turn(self, angle):
        """turn 

        :param angle: angle in degrees
        :type angle: int
        """
        
        if angle < 0: 
            angular_vel = -0.3 # NEG ANGLE MEANS RIGHT TURN
        else:
            angular_vel = 0.3
        time_to_sleep = (pi / angular_vel / 2) * (4) * (angle/360) # 90 degrees * 4 * (angle/360)
    
        self.move(0.0, angular_vel)
        # print(f"sleeping: {abs(time_to_sleep)}")
        sleep(abs(time_to_sleep))
        self.move(0.0,0.0)
        
        
    def run_loop(self):
        print(f"state: {self.state}")
        # self.turn(-2)

        if self.state is not None:
            if self.state == States.RIGHT:
                self.turn(-2)
            elif self.state == States.LEFT:
                self.turn(2)
            elif self.state == States.STRAIGHT:
                self.move(1.0, 0.0)
                sleep(0.5)

                
       
        
        
def main(args=None):
    rclpy.init(args=args) # init the node
    node = WallFollower()
    rclpy.spin(node) # starts up the node
    rclpy.shutdown() # if it finishes, it'll shutdown
    
if __name__ == '__main__':
    main()

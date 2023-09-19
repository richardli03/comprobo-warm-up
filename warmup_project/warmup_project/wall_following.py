import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pi
from enum import Enum, auto

CORRECTIVE_CONSTANT = 0.35


class States(Enum):
    AGGRESSIVE_LEFT = auto()
    LEFT = auto()
    RIGHT = auto()
    STRAIGHT = auto()


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follow")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_msg = self.create_subscription(LaserScan, "scan", self.run_loop, 10)

    def move(self, lin, ang):
        """Move the robot with some linear velocity and some angular velocity.

        Automatically converts the input to a float(as the Twist() message
        only accepts floats), then publishes a message (hopefully to /cmd_vel)
        that will move the robot accordingly.

        :param lin: the linear velocity to give the robot
        :type lin: int
        :param ang: _description_
        :type ang: _type_
        """
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.publisher.publish(msg)

    def run_loop(self, msg):
        """
        From a scan message, determine what state the system
        ought to exist in.

        :param msg: _description_
        :type msg: _type_
        """
        straight_ahead_dist = msg.ranges[0]
        theta1_dist = msg.ranges[45]
        theta2_dist = msg.ranges[135]  # 90 degrees past the previous
        # print(straight_aead_dist)
        print(f"THETA1 was {theta1_dist}, theta2_dist was {theta2_dist}")
        if straight_ahead_dist < 0.5:
            turn_mag = -5  # if we're stuck against a wall, turn until we're not
        else:
            turn_mag = theta1_dist - theta2_dist

        print(turn_mag)
        self.move(0.1, turn_mag)


def main(args=None):
    rclpy.init(args=args)  # init the node
    node = WallFollower()
    rclpy.spin(node)  # starts up the node
    rclpy.shutdown()  # if it finishes, it'll shutdown


if __name__ == "__main__":
    main()

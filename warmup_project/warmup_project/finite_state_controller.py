import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pi
from numpy import mean
from enum import Enum, auto
import numpy as np

# How many degrees in front of the robot to consider an object as "blocking"
VISION_CONE = 180


class States(Enum):
    AVOIDING = auto()
    FOLLOWING = auto()


class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__("finite_state_controller")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_msg = self.create_subscription(LaserScan, "scan", self.run_loop, 10)
        self.odom = self.create_subscription(Odometry, "odom", self.get_orientation, 10)
        self.orientation = None
        self.state = States.FOLLOWING

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

    def get_orientation(self, msg):
        """
        Find the current orientation of the robot and print it.
        Doesn't technically return anything, because I didn't use the output
        of this function.

        :param msg: a msg from the odometry output of the robot
        :type msg: Odometry
        """

        self.orientation = msg.pose.pose.orientation

    def run_loop(self, msg):
        """
        At every point in time, compute the mean index(!) of
        all the points visible to the neato that aren't above 1000.

        Based on the current state, determine whether or not we're
        trying to move towards the object of interest or move away
        from it. The robot both cannot get to close to the object or
        get far away from it -- how toxic!

        :param msg: message from the scanner
        :type msg: LaserScan
        """
        # print(msg.ranges)
        midpoint = len(msg.ranges) // 2

        # arbitrary v big number because it'll read infinity if too far
        results = sorted(
            [
                ((i + midpoint) % len(msg.ranges))
                for i, val in enumerate(msg.ranges)
                if val < 1000
            ]
        )
        forward_mag = 0.5
        # NOTE: results[0] = msg.ranges[180]
        avg_direction = mean(results)
        if self.state == States.AVOIDING:
            if avg_direction < 30 or avg_direction > 330:
                self.state = States.FOLLOWING
                turn_angle = avg_direction - 180
            elif avg_direction < (midpoint - VISION_CONE // 2) or avg_direction > (
                midpoint + VISION_CONE // 2
            ):
                turn_angle = 0
            else:
                turn_angle = avg_direction - 90

        if self.state == States.FOLLOWING:
            if msg.ranges[0] < 1:
                forward_mag = forward_mag * (msg.ranges[0]) * 0.5
                self.state = States.AVOIDING
            else:
                pass

            turn_angle = avg_direction - 180

        # magic number turning multiplier, just so the robot doesn't fishtail
        turn_mag = 0.05 * turn_angle
        # if avg_directione > 0:

        self.move(forward_mag, turn_mag)

        print(results)
        print(self.state)


def main(args=None):
    rclpy.init(args=args)  # init the node
    node = FiniteStateMachine()
    rclpy.spin(node)  # starts up the node
    rclpy.shutdown()  # if it finishes, it'll shutdown


if __name__ == "__main__":
    main()

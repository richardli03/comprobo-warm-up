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

FORWARD_SPEED = 0.5

# force the robot to drive slower so behavior is more controlled
DRIVE_MULTIPLIER = 0.5
TURNING_MULTIPLER = 0.05


class PersonFollower(Node):
    def __init__(self):
        super().__init__("person_follower")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_msg = self.create_subscription(LaserScan, "scan", self.run_loop, 10)

    def move(self, lin, ang):
        """Move the robot with some linear velocity and some angular velocity.

        Automatically converts the input to a float(as the Twist() message
        only accepts floats), then publishes a message (hopefully to /cmd_vel)
        that will move the robot accordingly.

        :param lin: the linear velocity to give the robot
        :type lin: int/float
        :param ang: the angular velocity to give the robot
        :type ang: int/float
        """
        msg = Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        self.publisher.publish(msg)

    def run_loop(self, msg):
        """
        At every point in time, compute the mean index(!) of
        all the points visible to the neato that aren't above 1000.

        Proportionally turn until the avg direction = 180 (which, in
        my reindexed list, is straight ahead). Additionally, drive
        forward proportionally to how close the object is.

        :param msg: _description_
        :type msg: _type_
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

        # NOTE: results[0] = msg.ranges[180]
        avg_direction = mean(results)
        forward_mag = FORWARD_SPEED
        print(msg.ranges[0])
        if msg.ranges[0] < 1:
            forward_mag = forward_mag * (msg.ranges[0]) * DRIVE_MULTIPLIER
        else:
            pass

        # "magic number", this turns the robot to face the object
        turn_angle = avg_direction - 180

        # turning multiplier, just so the robot doesn't fishtail
        turn_mag = TURNING_MULTIPLER * turn_angle
        self.move(forward_mag, turn_mag)

        print(results)


def main(args=None):
    rclpy.init(args=args)  # init the node
    node = PersonFollower()
    rclpy.spin(node)  # starts up the node
    rclpy.shutdown()  # if it finishes, it'll shutdown


if __name__ == "__main__":
    main()

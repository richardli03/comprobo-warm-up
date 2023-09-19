import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pi, radians, sin, cos
from enum import Enum, auto
from visualization_msgs.msg import Marker


class States(Enum):
    AGGRESSIVE_LEFT = auto()
    LEFT = auto()
    RIGHT = auto()
    STRAIGHT = auto()


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follow")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom = self.create_subscription(Odometry, "odom", self.update_position, 10)
        self.marker = self.create_publisher(Marker, "vis_mark", 10)
        self.scan_msg = self.create_subscription(LaserScan, "scan", self.run_loop, 10)

    def update_position(self, msg):
        """
        NOTE: while this isn't really used in this naive implementation of
        drive-square, I still found the data useful.

        Find the current position and orientation of the robot and print it.
        Doesn't technically return anything, because I didn't use the output
        of this function.

        :param msg: a msg from the odometry output of the robot
        :type msg: Odometry
        """
        self.xpos = msg.pose.pose.position.x
        self.ypos = msg.pose.pose.position.y
        self.zpos = msg.pose.pose.position.z

    def compute_marker_location(self, dist, angle):
        """
        Given a distance and angle, compute what the
        x,y coordinates for a marker ought to be

        :param dist: a distance
        :type dist: int
        :param angle: an angle from the global frame's 0 degrees
        :type angle: int
        :return: the x and y coordinate
        :rtype: tuple
        """
        return (
            (self.xpos + cos(radians(angle)) * dist),
            (self.ypos + sin(radians(angle)) * dist),
        )

    def pub_marker(self, dist, angle):
        """Publish the marker at 45 and 135 degrees that the robot uses to follow
        a wall. TODO: factor in robot orientation, not global!
        """
        msg = Marker()
        msg.header.frame_id = "odom"
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD

        msg.pose.position.x, msg.pose.position.y = self.compute_marker_location(
            dist, angle
        )
        msg.pose.position.z = 0.0

        print(msg.pose.position.x, msg.pose.position.y)

        msg.scale.x = 0.2
        msg.scale.y = 0.2
        msg.scale.z = 0.2

        msg.color.a = 1.0
        msg.color.r = 0.0
        msg.color.g = 0.0
        msg.color.b = 1.0
        self.marker.publish(msg)

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
        Proportionally turn the robot based off of how far off of parallel to the wall
        it is.

        :param msg: _description_
        :type msg: _type_
        """
        straight_ahead_dist = msg.ranges[0]
        theta1_dist = msg.ranges[45]
        theta2_dist = msg.ranges[135]  # 90 degrees past the previous
        # print(straight_aead_dist)
        forward_mag = 0.1
        # print(f"THETA1 was {theta1_dist}, theta2_dist was {theta2_dist}")
        if straight_ahead_dist < 0.5:
            turn_mag = -5  # if we're stuck against a wall, turn until we're not
        else:
            turn_mag = theta1_dist - theta2_dist

        # print(turn_mag)
        self.move(forward_mag, turn_mag)
        self.pub_marker(dist=theta1_dist, angle=45)
        self.pub_marker(dist=theta1_dist, angle=135)


def main(args=None):
    rclpy.init(args=args)  # init the node
    node = WallFollower()
    rclpy.spin(node)  # starts up the node
    rclpy.shutdown()  # if it finishes, it'll shutdown


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import sleep
from math import pi

# The turn computation appears to be a little incorrect.
# This constant fixes it -- I think I would ascribe
# the error to acceleration which isn't accounted for?
CORRECTIVE_CONSTANT = 0.35


class DriveSquareNode(Node):
    def __init__(self):
        super().__init__("square")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom = self.create_subscription(Odometry, "odom", self.update_position, 10)
        timer_period = 1
        # self.run_loop()
        self.timer = self.create_timer(timer_period, self.run_loop)

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

        self.orientation = msg.pose.pose.orientation
        print(self.xpos, self.ypos, self.zpos, self.orientation)

    def turn_left(self, angle):
        """Turn the robot some number of degrees to the left.

        :param angle: angle in degrees
        :type angle: int
        """
        angular_vel = 0.3
        time_to_sleep = (pi / angular_vel / 2) * (4) * (angle / 360)  # 90 degrees

        self.move(0.0, angular_vel)
        sleep(time_to_sleep + CORRECTIVE_CONSTANT)
        self.move(0.0, 0.0)

    def run_loop(self):
        for _ in range(4):
            self.move(1.0, 0.0)
            sleep(1)
            self.move(0.0, 0.0)
            sleep(0.5)
            self.turn_left(90)
            sleep(1)
        self.move(0.0, 0.0)
        self.destroy_timer(self.timer)


def main(args=None):
    rclpy.init(args=args)  # init the node
    node = DriveSquareNode()
    rclpy.spin(node)  # starts up the node
    rclpy.shutdown()  # if it finishes, it'll shutdown


if __name__ == "__main__":
    main()

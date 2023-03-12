#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Turtlebot3(Node):
    """
    A ROS2 node to move a Turtlebot3 robot and stop if an obstacle is within 0.5 meters.
    """
    def __init__(self):
        """
        Constructor.

        Initializes the node, publisher, and subscriber.
        """
        super().__init__('turtlebot3')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription_ = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.current_scan = None

    def scan_callback(self, msg):
        """
        Callback function to handle incoming laser scan messages.

        Saves the most recent scan message to `self.current_scan`.
        """
        self.current_scan = msg

    def move(self, linear_velocity, angular_velocity):
        """
        Publishes linear and angular velocities to the robot.

        Args:
            linear_velocity (float): The desired linear velocity.
            angular_velocity (float): The desired angular velocity.
        """
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.publisher_.publish(msg)

    def run(self):
        """
        Main loop of the node.

        Moves the robot forward if there are no obstacles within 0.5 meters, and stops the robot
        if there is an obstacle. Runs until the node is shut down.
        """
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.current_scan is not None:
                ranges = self.current_scan.ranges
                min_range = min(ranges)
                if min_range < 0.2:
                    # stop if obstacle is within 0.5 meters
                    self.move(0.0, 0.0)
                else:
                    # move forward if no obstacle detected
                    self.move(0.1, 0.0)
                self.current_scan = None

def main(args=None):
    """
    Main function to start the node.
    """
    rclpy.init(args=args)
    node = Turtlebot3()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

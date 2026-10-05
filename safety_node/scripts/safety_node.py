#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.ttc_threshold = 0.8

        #publisher /drive
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        #subscribe /scan
        self.scan_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10) 

        #subscribe /ego_racecar/odom
        self.odom_subscription = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # TODO: create ROS subscribers and publishers. DONE

    def odom_callback(self, odom_msg):
        # TODO: update current speed DONE
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):

        # TODO: calculate TTC
        ranges = np.array(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg._angle_increment #Generate angles corresponding to each beam: theta_i = angle_min + i * angle_increment

        valid = (
            np.isfinite(ranges)
            & (ranges >= scan_msg.range_min)
            & (ranges <= scan_msg.range_max)
        )

        # Range rate for each beam is -v*cos(theta); the closing speed is its
        # negative. Beams pointing away from the direction of travel have
        # closing speed <= 0, so they can never cause a collision.
        closing_speed = self.speed * np.cos(angles)
        mask = valid & (closing_speed > 1e-3)
        if not np.any(mask):
            return

        # Time to collision per beam: r / max(-r_dot, 0)
        ttc = ranges[mask] / closing_speed[mask]
        min_ttc = ttc.min()

        if min_ttc < self.ttc_threshold:
            self.get_logger().warn("Emergency brake.")
            brake_msg = AckermannDriveStamped()
            brake_msg.header.stamp = self.get_clock().now().to_msg()
            brake_msg.drive.speed = 0.0
            self.drive_publisher.publish(brake_msg)



        # TODO: publish command to brake


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
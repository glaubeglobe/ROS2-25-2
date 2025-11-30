#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class TwistToDrive(Node):
    def __init__(self):
        super().__init__('twist_to_drive')
        self.declare_parameter('wheelbase', 0.33)
        # rclpy 2 스타일
        self.wheelbase = self.get_parameter('wheelbase').value

        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10
        )
        self.get_logger().info(
            f"TwistToDrive started with wheelbase={self.wheelbase:.3f} m"
        )

    def cmd_callback(self, msg: Twist):
        v = msg.linear.x
        omega = msg.angular.z

        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'base_link'

        drive.drive.speed = float(v)

        if abs(omega) > 1e-6 and abs(v) > 1e-6:
            steer = math.atan2(self.wheelbase * omega, v)
        else:
            steer = 0.0

        drive.drive.steering_angle = float(steer)

        self.drive_pub.publish(drive)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
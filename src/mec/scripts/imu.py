#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64

import math


class ImuSplitter(Node):
    def __init__(self):
        super().__init__('imu_splitter')

        # Subscriber
        self.sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.pub_angular = self.create_publisher(
            Vector3,
            '/imu/angular_velocity',
            10
        )

        self.pub_linear = self.create_publisher(
            Vector3,
            '/imu/linear_acceleration',
            10
        )

        self.pub_orientation = self.create_publisher(
            Quaternion,
            '/imu/orientation',
            10
        )

        self.pub_yaw = self.create_publisher(
            Float64,
            '/imu/yaw',
            10
        )

        self.get_logger().info("IMU Splitter Node Started")

    def imu_callback(self, msg: Imu):

        # -------- Angular Velocity --------
        ang = Vector3()
        ang.x = msg.angular_velocity.x
        ang.y = msg.angular_velocity.y
        ang.z = msg.angular_velocity.z
        self.pub_angular.publish(ang)

        # -------- Linear Acceleration --------
        lin = Vector3()
        lin.x = msg.linear_acceleration.x
        lin.y = msg.linear_acceleration.y
        lin.z = msg.linear_acceleration.z
        self.pub_linear.publish(lin)

        # -------- Orientation (Quaternion) --------
        q = Quaternion()
        q.x = msg.orientation.x
        q.y = msg.orientation.y
        q.z = msg.orientation.z
        q.w = msg.orientation.w
        self.pub_orientation.publish(q)

        # -------- Yaw only --------
        yaw = self.quaternion_to_yaw(q)
        yaw_msg = Float64()
        yaw_msg.data = yaw
        self.pub_yaw.publish(yaw_msg)

    def quaternion_to_yaw(self, q: Quaternion):
        """
        Convert quaternion to yaw (Z axis rotation)
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

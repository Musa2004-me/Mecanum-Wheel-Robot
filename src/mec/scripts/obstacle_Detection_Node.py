#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class ObstacleDetector(Node):
    """
    Node that reads LiDAR and sets obstacle flags:
    last_front, last_left, last_right
    """

    def __init__(self):
        super().__init__('obstacle_detector')

        # Subscription
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Optional publishers for visualization
        self.pub_front = self.create_publisher(Bool, '/obstacle_front', 10)
        self.pub_left  = self.create_publisher(Bool, '/obstacle_left', 10)
        self.pub_right = self.create_publisher(Bool, '/obstacle_right', 10)

        # Thresholds
        self.front_thresh = 3.0
        self.side_thresh  = 1.5

        # Store last scan values
        self.last_front = False
        self.last_left  = False
        self.last_right = False

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        front, left, right = False, False, False

        for i, r in enumerate(ranges):
            if r < msg.range_min or r > msg.range_max:
                continue

            angle = angle_min + i * angle_inc
            deg = math.degrees(angle)

            # Front sector -15° to 15°
            if -15 <= deg <= 15 and r < self.front_thresh:
                front = True

            # Left sector 30° to 90°
            if 30 <= deg <= 90 and r < self.side_thresh:
                left = True

            # Right sector -90° to -30°
            if -90 <= deg <= -30 and r < self.side_thresh:
                right = True

        # Update last scan
        self.last_front = front
        self.last_left  = left
        self.last_right = right

        # Publish for visualization
        self.pub_front.publish(Bool(data=front))
        self.pub_left.publish(Bool(data=left))
        self.pub_right.publish(Bool(data=right))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
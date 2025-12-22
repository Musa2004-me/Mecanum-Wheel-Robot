#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math


class LaneDetection(Node):

    def __init__(self):
        super().__init__('lane_detection_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.dist_pub = self.create_publisher(
            Float32,
            '/lane_error_dist',
            10
        )

        self.slope_pub = self.create_publisher(
            Float32,
            '/lane_error_slope',
            10
        )

        self.get_logger().info("Lane Detection (Dist + Slope) Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        height, width, _ = frame.shape
        image_center = width // 2

        # ========== HSV ==========
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ----- WHITE MASK -----
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        # ----- YELLOW MASK -----
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([40, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Combine masks
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # ========== ROI ==========
        roi = np.zeros_like(lane_mask)
        roi[int(height * 0.6):height, :] = 255
        lane_mask = cv2.bitwise_and(lane_mask, roi)

        # ========== MORPHOLOGY ==========
        kernel = np.ones((5, 5), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)

        # ========== LATERAL ERROR ==========
        M = cv2.moments(lane_mask)

        if M["m00"] > 0:
            lane_center = int(M["m10"] / M["m00"])
            dist_error = float(lane_center - image_center)

            msg_dist = Float32()
            msg_dist.data = dist_error
            self.dist_pub.publish(msg_dist)

            # Draw center lines
            cv2.line(frame, (lane_center, height),
                     (lane_center, int(height * 0.6)), (0, 255, 255), 3)
            cv2.line(frame, (image_center, height),
                     (image_center, int(height * 0.6)), (255, 255, 255), 2)

        else:
            self.get_logger().warn("Lane not detected")
            return

        # ========== SLOPE / HEADING ERROR ==========
        def band_center_x(mask):
            M = cv2.moments(mask)
            if M["m00"] > 0:
                return int(M["m10"] / M["m00"])
            return None

        top_y1 = int(height * 0.65)
        top_y2 = int(height * 0.75)
        bottom_y1 = int(height * 0.85)
        bottom_y2 = height

        top_band = lane_mask[top_y1:top_y2, :]
        bottom_band = lane_mask[bottom_y1:bottom_y2, :]

        x_top = band_center_x(top_band)
        x_bottom = band_center_x(bottom_band)

        if x_top is not None and x_bottom is not None:
            dx = x_top - x_bottom
            dy = bottom_y1 - top_y2

            slope_error = math.atan2(dx, dy)

            msg_slope = Float32()
            msg_slope.data = slope_error
            self.slope_pub.publish(msg_slope)

            # Draw heading line
            cv2.line(
                frame,
                (x_bottom, bottom_y1),
                (x_top, top_y2),
                (0, 0, 255),
                3
            )

            # self.get_logger().info(
            #     f"Dist error: {dist_error:.2f} px | Slope error: {slope_error:.4f} rad"
            # )

        if "DISPLAY" in os.environ:
            cv2.imshow("Lane Mask", lane_mask)
            cv2.imshow("Lane Detection", frame)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = LaneDetection()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import math
import numpy as np


class SequentialLaneController(Node):

    def __init__(self):
        super().__init__('sequential_lane_controller')

        # Subscribers
        self.create_subscription(Float32, '/lane_error_dist', self.dist_cb, 10)
        self.create_subscription(Float32, '/lane_error_slope', self.slope_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # State
        self.state = "ALIGN_YAW_INIT"
        self.lane_dist = 0.0
        self.lane_slope = 0.0
        self.x_pos = 0.0
        self.x_error = 0.13
        # --- FIX: HYSTERESIS LIMITS ---
        # We trigger correction at a high threshold, but don't stop until we reach a low threshold.
        self.slope_start_limit = math.radians(10) # Trigger yaw fix if > 10 deg
        self.slope_goal = math.radians(3)        # Stop yaw fix when < 3 deg
        
        self.dist_start_limit = 40.0             # Trigger lateral fix if > 40px
        self.dist_goal = 20.0                    # Stop lateral fix when < 10px

        # Speeds
        self.yaw_speed = 0.05
        self.lateral_speed = 0.2
        self.forward_speed = 1.1
        self.max_px_error = 100.0 # Used for scaling speed
        self.target_x = 10.0 - self.x_error

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ Sequential Lane Controller Started with Hysteresis Fix")

    # Callbacks
    def dist_cb(self, msg):
        self.lane_dist = msg.data

    def slope_cb(self, msg):
        self.lane_slope = msg.data

    def odom_cb(self, msg):
        self.x_pos = msg.pose.pose.position.x

    # Control loop
    def control_loop(self):
        cmd = Twist()

        # 0. Global Stop Condition
        if self.x_pos >= self.target_x:
            self.state = "STOP"

        if self.state == "STOP":
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("🛑 Target reached → STOP")
            return

        # 1. INITIAL YAW ALIGNMENT
        if self.state == "ALIGN_YAW_INIT":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            max_yaw_speed = 0.5
            yaw_gain = 0.5

            # Fix until we hit the 'goal' (tight tolerance)
            if abs(self.lane_slope) > self.slope_goal:
                cmd.angular.z = yaw_gain * self.lane_slope
                cmd.angular.z = np.clip(cmd.angular.z, -max_yaw_speed, max_yaw_speed)
                self.get_logger().info(f"🔄 Aligning Yaw: slope={math.degrees(self.lane_slope):.2f} deg")
            else:
                cmd.angular.z = 0.0
                self.state = "ALIGN_LATERAL_INIT"
                self.get_logger().info("✅ Yaw aligned → ALIGN_LATERAL_INIT")

        # 2. INITIAL LATERAL ALIGNMENT
        elif self.state == "ALIGN_LATERAL_INIT":
            cmd.linear.x = 0.3  # Slow creep forward while sliding sideways
            cmd.angular.z = 0.0
            # Scale side speed based on distance
            error_ratio = np.clip(self.lane_dist / self.max_px_error, -1.0, 1.0)

            # Fix until we hit the 'goal' (tight tolerance)
            if abs(self.lane_dist) > self.dist_goal:
                cmd.linear.y = -self.lateral_speed * error_ratio
                self.get_logger().info(f"🔄 Aligning Lateral: dist={self.lane_dist:.1f} px")
            else:
                cmd.linear.y = 0.0
                self.state = "READY"
                self.get_logger().info("✅ Lateral aligned → READY")

        # 3. READY / FORWARD MOTION
        elif self.state == "READY":
            cmd.linear.x = self.forward_speed
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            # ONLY revert to alignment if error crosses the 'start_limit' (loose tolerance)
            if abs(self.lane_dist) > self.dist_start_limit:
                self.get_logger().warn(f"⚠️ Drift detected ({self.lane_dist:.1f})! Re-aligning...")
                self.state = "ALIGN_LATERAL_INIT"
            
            # elif abs(self.lane_slope) > self.slope_start_limit:
            #     self.get_logger().warn(f"⚠️ Heading drift! Re-aligning Yaw...")
            #     self.state = "ALIGN_YAW_INIT"

        self.cmd_pub.publish(cmd)
        # Log status for debugging
        if self.state != "READY" or int(self.x_pos * 10) % 10 == 0: # Reduce log spam in READY state
            self.get_logger().info(
                f"STATE={self.state} | x={self.x_pos:.2f} | dist={self.lane_dist:.1f} | slope={math.degrees(self.lane_slope):.2f} deg"
            )


def main():
    rclpy.init()
    node = SequentialLaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
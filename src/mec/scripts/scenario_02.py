#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
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
        self.create_subscription(Bool, '/obstacle_front', self.front_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # State
        self.state = "ALIGN_YAW_INIT"
        self.lane_dist = 0.0
        self.lane_slope = 0.0
        self.x_pos = 0.0
        self.obs_front = False
        self.current_lane = "RIGHT"  # يبدأ من الحارة اليمنى
        self.obs_counter = 0
        self.last_obs_front = False # To detect the "Edge" (True -> False)
        
        # Lane positions (مثلاً المسافة بين الحارات 0.3 متر)
        self.lane_offset = 0.3  # meters
        self.target_lane_dist = 0.0  # هدف الحارة الجديدة

        # Limits
        self.slope_start_limit = math.radians(10)
        self.slope_goal = math.radians(10)
        self.dist_start_limit = 30.0
        self.dist_goal = 20.0
        self.x_error = 0.07
        # Speeds
        self.yaw_speed = 0.05
        self.lateral_speed = 0.2
        self.forward_speed = 1.1
        self.max_px_error = 100.0
        self.target_x = 10.0 - self.x_error

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ Sequential Lane Controller Started")

    # Callbacks
    def dist_cb(self, msg):
        self.lane_dist = msg.data

    def slope_cb(self, msg):
        self.lane_slope = msg.data

    def odom_cb(self, msg):
        self.x_pos = msg.pose.pose.position.x

    def front_cb(self, msg):
        self.obs_front = msg.data

    # Control loop
    def control_loop(self):
        cmd = Twist()

        # Stop at target
        if self.x_pos >= self.target_x:
            self.state = "STOP"

        if self.state == "STOP":
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("🛑 Target reached → STOP")
            return

        # ===== INITIAL YAW ALIGNMENT =====
        if self.state == "ALIGN_YAW_INIT":
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            max_yaw_speed = 0.5
            yaw_gain = 0.5

            if abs(self.lane_slope) > self.slope_start_limit:
                cmd.angular.z = yaw_gain * self.lane_slope
                cmd.angular.z = np.clip(cmd.angular.z, -max_yaw_speed, max_yaw_speed)
                self.get_logger().info(f"🔄 Aligning Yaw: slope={math.degrees(self.lane_slope):.2f} deg")
            elif abs(self.lane_slope) <= self.slope_goal:
                cmd.angular.z = 0.0
                self.state = "ALIGN_LATERAL_INIT"
                self.get_logger().info("✅ Yaw aligned → ALIGN_LATERAL_INIT")

        # ===== INITIAL LATERAL ALIGNMENT =====
        elif self.state == "ALIGN_LATERAL_INIT":
            cmd.linear.x = 0.2  # Slow creep forward while sliding sideways
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

        # ===== READY TO MOVE FORWARD =====
        elif self.state == "READY":
            if self.obs_front:
                if self.current_lane == "RIGHT":
                    cmd.linear.x = self.forward_speed
                    cmd.linear.y = 0.3
                    cmd.angular.z = 0.0
                else:
                    cmd.linear.x = self.forward_speed
                    cmd.linear.y = -0.3
                    cmd.angular.z = 0.0


            else:
                # if self.lane_dist > 40.0:  # Threshold where robot is safely in left lane
                #     if self.current_lane == "RIGHT":
                #         self.current_lane = "LEFT"
                #     else:
                #         self.current_lane = "RIGHT"
                cmd.linear.x = self.forward_speed
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0

            # Continuous check for deviations
                # if abs(self.lane_slope) > self.slope_start_limit:
                #     self.state = "ALIGN_YAW_INIT"
                if abs(self.lane_dist) > self.dist_start_limit:
                    self.state = "ALIGN_LATERAL_INIT"

        # ===== MOVE TO OTHER LANE =====
        if self.last_obs_front == True and self.obs_front == False:
            self.obs_counter += 1
            self.get_logger().info(f"🚦 Obstacle Cleared! Counter: {self.obs_counter}")
        
        # Update last state for next iteration
        self.last_obs_front = self.obs_front

        # --- 2. Determine Target Lane based on Counter ---
        if self.obs_counter % 2 != 0:
            self.current_lane = "LEFT"
        else:
            self.current_lane = "RIGHT"
        # Publish command
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"STATE={self.state} | x={self.x_pos:.2f} | dist={self.lane_dist:.2f} |counter {self.obs_counter} | front_obs={self.obs_front} | lane={self.current_lane}"
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
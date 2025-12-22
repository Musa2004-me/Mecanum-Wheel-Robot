#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MecanumMotion(Node):
    """
    Mecanum motion controller
    - Subscribes to /cmd_vel_raw
    - Smooths velocity changes
    - Publishes /cmd_vel
    """

    def __init__(self):
        super().__init__('mecanum_motion')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_callback, 10
        )

        # Current velocity
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0

        # Target velocity
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0

        # Acceleration limits (per cycle)
        self.accel_linear = 0.025     # m/s per 0.05s
        self.accel_angular = 0.1      # rad/s per 0.05s
        
        #Deceleration limits (per cycle)
        self.decel_linear = 0.15      # m/s per 0.05s        
        self.decel_angular = 1.0      # rad/s per 0.05s

        self.timer = self.create_timer(0.05, self.update_velocity)

        self.get_logger().info("MecanumMotion with accel/decel started")

    # ================= CALLBACK =================
    def cmd_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_wz = msg.angular.z

    # ================= UPDATE =================
    def update_velocity(self):

        self.current_vx = self.ramp(
            self.current_vx,
            self.target_vx,
            self.accel_linear,
            self.decel_linear
        )

        self.current_vy = self.ramp(
            self.current_vy,
            self.target_vy,
            self.accel_linear,
            self.decel_linear
        )

        self.current_wz = self.ramp(
            self.current_wz,
            self.target_wz,
            self.accel_angular,
            self.decel_angular
        )

        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.angular.z = self.current_wz
        self.pub.publish(msg)

    # ================= UTILS =================
    @staticmethod
    def ramp(current, target, accel_step, decel_step):
        """
        accel_step: used when speeding up
        decel_step: used when slowing down
        """

        if target > current:
            return min(current + accel_step, target)

        elif target < current:
            return max(current - decel_step, target)

        return current


def main():
    rclpy.init()
    node = MecanumMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

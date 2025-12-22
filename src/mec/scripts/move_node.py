#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class KeyboardTeleop(Node):
    """
    Keyboard teleoperation node
    Publishes raw Twist commands to /cmd_vel_raw
    """

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # ====== SPEED SETTINGS ======
        self.linear_speed_x = 0.6   # forward / backward (faster)
        self.linear_speed_y = 0.2   # strafing (slower)
        self.angular_speed = 0.05   # rotation

        self.get_logger().info("Keyboard Teleop Started")
        self.get_logger().info(
            "W/S: forward/back | A/D: strafe | Q/E: rotate | "
            "Z/X/C/V: diagonals | SPACE or K: stop | CTRL+C: exit"
        )

    def get_key(self):
        """Read single key without Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        msg = Twist()

        try:
            while rclpy.ok():
                key = self.get_key()

                # Reset command
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0

                # ========== LINEAR ==========
                if key.lower() == 'w':
                    msg.linear.x = self.linear_speed_x
                elif key.lower() == 's':
                    msg.linear.x = -self.linear_speed_x
                elif key.lower() == 'a':
                    msg.linear.y = self.linear_speed_y
                elif key.lower() == 'd':
                    msg.linear.y = -self.linear_speed_y

                # ========== ROTATION ==========
                elif key.lower() == 'q':
                    msg.angular.z = self.angular_speed
                elif key.lower() == 'e':
                    msg.angular.z = -self.angular_speed

                # ========== DIAGONALS ==========
                elif key.lower() == 'z':
                    msg.linear.x = self.linear_speed_x
                    msg.linear.y = self.linear_speed_y
                elif key.lower() == 'x':
                    msg.linear.x = self.linear_speed_x
                    msg.linear.y = -self.linear_speed_y
                elif key.lower() == 'c':
                    msg.linear.x = -self.linear_speed_x
                    msg.linear.y = self.linear_speed_y
                elif key.lower() == 'v':
                    msg.linear.x = -self.linear_speed_x
                    msg.linear.y = -self.linear_speed_y

                # ========== STOP ==========
                elif key == ' ' or key.lower() == 'k':
                    pass  # already zero

                # ========== EXIT ==========
                elif key == '\x03':  # CTRL+C
                    break

                self.pub.publish(msg)

        except KeyboardInterrupt:
            pass
        finally:
            self.pub.publish(Twist())
            self.get_logger().info("Keyboard Teleop stopped")


def main():
    rclpy.init()
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MecanumOdometryNode(Node):
    def __init__(self):
        super().__init__('mecanum_odometry')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('lx', 0.253 / 2.0)
        self.declare_parameter('ly', 0.29 / 2.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_topic', '/imu/yaw')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_topic = self.get_parameter('imu_topic').value

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_time = None
        self.prev_positions = {
            "front_left_wheel_joint": None,
            "front_right_wheel_joint": None,
            "rear_left_wheel_joint": None,
            "rear_right_wheel_joint": None
        }

        # ---------------- PUB / SUB ----------------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 20)
        self.create_subscription(Float64, self.imu_topic, self.imu_cb, 20)

        self.get_logger().info("✅ Mecanum Odometry Node Started")

    # ---------------- IMU CALLBACK ----------------
    def imu_cb(self, msg: Float64):
        self.yaw = msg.data

    # ---------------- JOINT STATES CALLBACK ----------------
    def joint_states_cb(self, msg: JointState):
        now = rclpy.time.Time.from_msg(msg.header.stamp)

        if self.prev_time is None:
            self.prev_time = now
            for i, name in enumerate(msg.name):
                if name in self.prev_positions:
                    self.prev_positions[name] = msg.position[i]
            return

        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        self.prev_time = now

        # -------- compute wheel angular velocities --------
        wheel_vel = {}

        for i, name in enumerate(msg.name):
            if name in self.prev_positions:
                prev = self.prev_positions[name]
                if prev is not None:
                    wheel_vel[name] = (msg.position[i] - prev) / dt
                else:
                    wheel_vel[name] = 0.0
                self.prev_positions[name] = msg.position[i]

        # Ensure all wheels exist
        for key in self.prev_positions:
            wheel_vel.setdefault(key, 0.0)

        # -------- explicit mapping (CRITICAL FIX) --------
        w_fl = wheel_vel["front_left_wheel_joint"]
        w_fr = wheel_vel["front_right_wheel_joint"]
        w_rl = wheel_vel["rear_left_wheel_joint"]
        w_rr = wheel_vel["rear_right_wheel_joint"]

        R = self.wheel_radius
        L = self.lx
        W = self.ly

        # -------- mecanum kinematics --------
        vx = (R / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy = (R / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        wz = (R / (4.0 * (L + W))) * (-w_fl + w_fr - w_rl + w_rr)

        # -------- integrate pose --------
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt

        self.x += dx
        self.y += dy

        # -------- publish odometry --------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quaternion_from_yaw(self.yaw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # -------- TF --------
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = quaternion_from_yaw(self.yaw)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MecanumDriverNode(Node):
    def __init__(self):
        super().__init__('mecanum_driver')

        # --- Parameters (from YAML) ---
        self.declare_parameter("wheel_radius", 0.0485)
        self.declare_parameter("lx", 0.253 / 2.0)
        self.declare_parameter("ly", 0.29 / 2.0)
        self.declare_parameter("control_mode", "open")   # open / closed

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.lx = self.get_parameter("lx").value
        self.ly = self.get_parameter("ly").value
        self.control_mode = self.get_parameter("control_mode").value
        self.lxly_sum = self.lx + self.ly

        # --- Joint Names ---
        self.joint_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"
        ]
        # --- Roller Joint Names (Passive/Dummy) ---
        # These are the joints causing the "No Transform" error.
        # We will publish 0.0 for them continuously.
        self.roller_joint_names = [
            'roller_14_1_joint', 'roller_16_1_joint', 'roller_13_1_joint', 'roller_12_1_joint', 
            'roller_15_1_joint', 'roller_17_1_joint', 'roller_18_1_joint', 'roller_10_1_joint', 
            'roller_11_1_joint', 'roller_7_1_joint', 'roller_5_1_joint', 'roller_3_1_joint', 
            'roller_6_1_joint', 'roller_8_1_joint', 'roller_9_1_joint', 'roller_4_1_joint', 
            'roller_1_1_joint', 'roller_2_1_joint', 'roller_23_1_joint', 'roller_22_1_joint', 
            'roller_25_1_joint', 'roller_21_1_joint', 'roller_26_1_joint', 'roller_27_1_joint', 
            'roller_20_1_joint', 'roller_19_1_joint', 'roller_24_1_joint', 'roller_34_1_joint', 
            'roller_30_1_joint', 'roller_32_1_joint', 'roller_31_1_joint', 'roller_29_1_joint', 
            'roller_28_1_joint', 'roller_35_1_joint', 'roller_36_1_joint', 'roller_33_1_joint'
        ]


        # --- Target + PID output ---
        self.target_speed = {j: 0.0 for j in self.joint_names}
        self.pid_output = {j: 0.0 for j in self.joint_names}

        # --- Publishers to controllers ---
        self.pubs = {
            "front_left_wheel_joint":  self.create_publisher(Float64MultiArray, "/front_left_wheel_controller/commands", 10),
            "front_right_wheel_joint": self.create_publisher(Float64MultiArray, "/front_right_wheel_controller/commands", 10),
            "rear_left_wheel_joint":   self.create_publisher(Float64MultiArray, "/rear_left_wheel_controller/commands", 10),
            "rear_right_wheel_joint":  self.create_publisher(Float64MultiArray, "/rear_right_wheel_controller/commands", 10)
        }
        

        # --- Publisher for PID node ---
        self.target_pub = self.create_publisher(Float64MultiArray, "/target_speed", 10)
        # --- Publisher for Rollers (Joint States) ---
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # --- Subscribers ---
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, "/pid_output", self.pid_output_callback, 10)

        # --- Timer Loop ---
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"Mecanum Driver Node started in [{self.control_mode}] mode")

    # ----------------------------------------------------
    #                   CALLBACKS
    # ----------------------------------------------------

    def pid_output_callback(self, msg):
        # msg.data = [fl, fr, rl, rr]
        for i, j in enumerate(self.joint_names):
            self.pid_output[j] = msg.data[i]

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        R = self.wheel_radius
        L = self.lxly_sum

        # inverse kinematics
        self.target_speed["front_left_wheel_joint"]  = (1/R)*(vx - vy - L*wz)
        self.target_speed["front_right_wheel_joint"] = (1/R)*(vx + vy + L*wz)
        self.target_speed["rear_left_wheel_joint"]   = (1/R)*(vx + vy - L*wz)
        self.target_speed["rear_right_wheel_joint"]  = (1/R)*(vx - vy + L*wz)

        # publish to PID node
        msg_out = Float64MultiArray()
        msg_out.data = [
            self.target_speed["front_left_wheel_joint"],
            self.target_speed["front_right_wheel_joint"],
            self.target_speed["rear_left_wheel_joint"],
            self.target_speed["rear_right_wheel_joint"]
        ]
        self.target_pub.publish(msg_out)

    # ----------------------------------------------------
    #                 CONTROL LOOP
    # ----------------------------------------------------

    def control_loop(self):
        for j in self.joint_names:

            if self.control_mode == "open":
                cmd = self.target_speed[j]
            else:
                cmd = self.pid_output[j]

            # clamp to safe range
            cmd = max(min(cmd, 200.0), -200.0)

            msg = Float64MultiArray()
            msg.data = [cmd]
            self.pubs[j].publish(msg)
        # 2. Handle Passive Rollers (Fix TF Errors)
        # We create a JointState message saying all rollers are at angle 0.0
        roller_msg = JointState()
        roller_msg.header.stamp = self.get_clock().now().to_msg()
        roller_msg.name = self.roller_joint_names
        roller_msg.position = [0.0] * len(self.roller_joint_names)
        
        # We leave velocity and effort empty as they are not needed for TF
        self.joint_state_pub.publish(roller_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
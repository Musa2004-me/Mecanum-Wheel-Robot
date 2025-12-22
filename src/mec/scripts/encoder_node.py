# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray
# import random

# class EncoderNode(Node):
#     def __init__(self):
#         super().__init__("encoder_node")

#         # Noise parameter
#         self.declare_parameter("noise_std", 0.0)  # يمكن تغييره من yaml
#         self.noise_std = self.get_parameter("noise_std").value

#         # ترتيب العجلات
#         self.joint_order = [
#             "front_left_wheel_joint",
#             "front_right_wheel_joint",
#             "rear_left_wheel_joint",
#             "rear_right_wheel_joint"
#         ]

#         # Sub to joint_states
#         self.sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
#         # Pub للسرعات الفعلية
#         self.pub = self.create_publisher(Float64MultiArray, "/actual_speed", 10)

#     def joint_state_callback(self, msg):
#         speeds = [0.0, 0.0, 0.0, 0.0]
#         for i, name in enumerate(msg.name):
#             if name in self.joint_order:
#                 idx = self.joint_order.index(name)
#                 actual = msg.velocity[i]
#                 noisy = actual + random.gauss(0, self.noise_std)
#                 speeds[idx] = noisy
#         out = Float64MultiArray()
#         out.data = speeds
#         self.pub.publish(out)


# def main(args=None):
#     rclpy.init(args=args)
#     node = EncoderNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray

# class PIDNode(Node):
#     def __init__(self):
#         super().__init__("pid_node")

#         # ترتيب العجلات
#         self.joint_order = [
#             "front_left_wheel_joint",
#             "front_right_wheel_joint",
#             "rear_left_wheel_joint",
#             "rear_right_wheel_joint"
#         ]

#         # PID parameters
#         self.declare_parameter("p", 1.0)
#         self.declare_parameter("i", 0.0)
#         self.declare_parameter("d", 0.0)

#         self.Kp = self.get_parameter("p").value
#         self.Ki = self.get_parameter("i").value
#         self.Kd = self.get_parameter("d").value

#         # State
#         self.target_speed = [0.0] * 4
#         self.actual_speed = [0.0] * 4
#         self.prev_error = [0.0] * 4
#         self.integral = [0.0] * 4

#         # Subscribers
#         self.create_subscription(Float64MultiArray, "/target_speed", self.target_callback, 10)
#         self.create_subscription(Float64MultiArray, "/actual_speed", self.actual_callback, 10)

#         # Publisher
#         self.pub = self.create_publisher(Float64MultiArray, "/pid_output", 10)

#         # Timer loop
#         self.timer = self.create_timer(0.05, self.control_loop)

#     def target_callback(self, msg):
#         self.target_speed = msg.data

#     def actual_callback(self, msg):
#         self.actual_speed = msg.data

#     def control_loop(self):
#         dt = 0.05  # نفس زمن الـ timer في driver
#         output = [0.0] * 4
#         for i in range(4):
#             error = self.target_speed[i] - self.actual_speed[i]
#             self.integral[i] += error * dt
#             derivative = (error - self.prev_error[i]) / dt
#             output[i] = self.Kp*error + self.Ki*self.integral[i] + self.Kd*derivative
#             self.prev_error[i] = error

#         msg_out = Float64MultiArray()
#         msg_out.data = output
#         self.pub.publish(msg_out)

# def main(args=None):
#     rclpy.init(args=args)
#     node = PIDNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

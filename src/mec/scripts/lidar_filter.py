# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import numpy as np


# class LaserFilters(Node):
#     def __init__(self):
#         super().__init__('laser_filters')

#         # Subscriber
#         self.sub = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.scan_callback,
#             10
#         )

#         # Publishers
#         self.pub_avg = self.create_publisher(LaserScan, '/scan_avg', 10)
#         self.pub_median = self.create_publisher(LaserScan, '/scan_median', 10)
#         self.pub_thresh = self.create_publisher(LaserScan, '/scan_thresh', 10)
#         self.pub_low = self.create_publisher(LaserScan, '/scan_lowpass', 10)
#         self.pub_kalman = self.create_publisher(LaserScan, '/scan_kalman', 10)

#         # Buffers
#         self.window_size = 5
#         self.buffer = []

#         # Low-pass filter
#         self.alpha = 0.4
#         self.lowpass_prev = None

#         # Kalman filter
#         self.kalman_prev = None
#         self.Q = 0.01
#         self.R = 0.05

#         self.get_logger().info("Laser filters node with 5 filters started...")


#     def scan_callback(self, msg: LaserScan):
#         ranges = np.array(msg.ranges)

#         avg_scan = self.moving_average_filter(ranges.copy())
#         median_scan = self.median_filter(ranges.copy())
#         thresh_scan = self.threshold_filter(ranges.copy(), min_range=0.15, max_range=msg.range_max)
#         lowpass_scan = self.lowpass_filter(ranges.copy())
#         kalman_scan = self.kalman_filter(ranges.copy())

#         self.publish_filtered(msg, avg_scan, self.pub_avg)
#         self.publish_filtered(msg, median_scan, self.pub_median)
#         self.publish_filtered(msg, thresh_scan, self.pub_thresh)
#         self.publish_filtered(msg, lowpass_scan, self.pub_low)
#         self.publish_filtered(msg, kalman_scan, self.pub_kalman)


#     # ======== FILTERS ========
#     def moving_average_filter(self, data):
#         self.buffer.append(data)
#         if len(self.buffer) > self.window_size:
#             self.buffer.pop(0)
#         return np.mean(self.buffer, axis=0)

#     def median_filter(self, data):
#         self.buffer.append(data)
#         if len(self.buffer) > self.window_size:
#             self.buffer.pop(0)
#         return np.median(self.buffer, axis=0)

#     def threshold_filter(self, data, min_range, max_range):
#         return np.where((data < min_range) | (data > max_range), np.nan, data)

#     def lowpass_filter(self, data):
#         if self.lowpass_prev is None:
#             self.lowpass_prev = data
#             return data
#         filtered = self.alpha * data + (1 - self.alpha) * self.lowpass_prev
#         self.lowpass_prev = filtered
#         return filtered

#     def kalman_filter(self, data):
#         if self.kalman_prev is None:
#             self.kalman_prev = data
#             return data
#         pred = self.kalman_prev
#         P = self.Q + self.R
#         K = P / (P + self.R)
#         updated = pred + K * (data - pred)
#         self.kalman_prev = updated
#         return updated

#     # ======== PUBLISHER ========
#     def publish_filtered(self, original_msg, new_ranges, publisher):
#         filtered_msg = LaserScan()
#         filtered_msg.header = original_msg.header
#         filtered_msg.angle_min = original_msg.angle_min
#         filtered_msg.angle_max = original_msg.angle_max
#         filtered_msg.angle_increment = original_msg.angle_increment
#         filtered_msg.time_increment = original_msg.time_increment
#         filtered_msg.scan_time = original_msg.scan_time
#         filtered_msg.range_min = original_msg.range_min
#         filtered_msg.range_max = original_msg.range_max
#         filtered_msg.ranges = new_ranges.tolist()
#         publisher.publish(filtered_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LaserFilters()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

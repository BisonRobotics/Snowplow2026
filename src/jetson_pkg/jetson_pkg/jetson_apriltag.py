import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from jetson_pkg.apriltag_interpretation import apriltag_interpretation

import math
import numpy as np
import threading
import cv2
import apriltag

class ApriltagPublisher(Node):
    def __init__(self):
        super().__init__("apriltag_publisher")
        self.publisher_ = self.create_publisher(Twist, "/apriltag", 10)
        timer_period = 0.5
        self.latest_frame = None

        # Declare parameters with default values
        self.declare_parameter('cap', '')
        self.declare_parameter('fx', 1.0)
        self.declare_parameter('fy', 1.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        # Assign parameters
        cap_value = self.get_parameter('cap').value
        self.cap = cv2.VideoCapture(cap_value) if cap_value else None
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error("Failed to open video capture device. Please check the connection string.")
            raise ValueError("Failed to open video capture device. Please check the connection string.")

        self.detector = apriltag.Detector()  # Initialize the detector
        threading.Thread(target=self.keep_up_thread, daemon=True).start()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def keep_up_thread(self):
        while True:
            if self.cap is not None and self.cap.isOpened():
                success, frame = self.cap.read()
                if success:
                    self.latest_frame = frame
                else:
                    self.get_logger().warning("Failed to read frame from video capture device.")

    def timer_callback(self):
        if self.latest_frame is None:
            self.get_logger().warning("No frame available for processing.")
            return

        try:
            gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2GRAY)
            detections = self.detector.detect(gray)
            if len(detections) > 0:
                pose, _, _ = self.detector.detection_pose(detections[0], [self.fx, self.fy, self.cx, self.cy], 0.3254375)
                relative_x = pose[0][3] + -0.017
                relative_z = pose[2][3] + 0.83
                relative_rotation = np.arcsin(-pose[2][0]) * (180 / math.pi)
                xr, zr, thetar = apriltag_interpretation(0, 3.71, 270, relative_x, relative_z, relative_rotation)
                msg = Twist()
                msg.linear.x = xr
                msg.linear.z = zr
                msg.angular.y = thetar
                self.get_logger().info(f'{msg}')
                self.publisher_.publish(msg)
        except cv2.error as e:
            self.get_logger().error(f"OpenCV error: {e}")

def main(args=None):
    rclpy.init(args=args)
    apriltag_publisher = ApriltagPublisher()
    rclpy.spin(apriltag_publisher)

    apriltag_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

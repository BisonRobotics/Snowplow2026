import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from jetson_pkg.apriltag_interpretation import apriltag_interpretation

import math
import numpy as np
import cv2
import apriltag

class ApriltagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')
        self.oldpublisher_ = self.create_publisher(Twist, '/old_apriltag', 10)
        self.publisher_ = self.create_publisher(Twist, '/apriltag', 10)
        timer_period = 0.5
        self.latest_frame = None

        # Declare parameters with default values
        self.declare_parameter('camera_name', '')

        # Assign parameters
        camera_name = self.get_parameter('camera_name').value
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.detector = apriltag.Detector()  # Initialize the detector
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.create_subscription(Image, f'{camera_name}/img', self.image_callback)
        self.create_subscription(CameraInfo, f'{camera_name}/cam_info', self.set_cam_info)
    
    def image_callback(self, msg):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def set_cam_info(self):
        self.fx = CameraInfo.k[0]
        self.fy = CameraInfo.k[4]
        self.cx = CameraInfo.k[2]
        self.cy = CameraInfo.k[5]

        self.get_logger().info(f"Camera fx set to: {self.fx}")
        self.get_logger().info(f"Camera fy set to: {self.fy}")
        self.get_logger().info(f"Camera cx set to: {self.cx}")
        self.get_logger().info(f"Camera cy set to: {self.cy}")
    
    def timer_callback(self):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().error("Camera intrinsic parameters (fx, fy, cx, cy) are not set.")
            return

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
                self.oldpublisher_.publish(msg)
                #new without math
                relative_x = pose[0][3]
                relative_y = pose[1][3]
                relative_z = pose[2][3]

                msg_no_offset = Twist()
                msg_no_offset.linear.x = relative_x
                msg_no_offset.linear.z = relative_z
                msg_no_offset.linear.y = relative_y
                
                self.get_logger().debug(f'{msg_no_offset}')
                self.publisher_.publish(msg_no_offset)
        except cv2.error as e:
            self.get_logger().error(f"OpenCV error: {e}")
        
def main(args=None):
    rclpy.init(args=args)
    apriltag_publisher = ApriltagPublisher()
    rclpy.spin(apriltag_publisher)
    
    apriltag_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
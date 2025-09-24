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
        super().__init__('apriltag_publisher')
        self.publisher_ = self.create_publisher(Twist, '/apriltag', 10)
        timer_period = 0.5
        self.latest_frame = None
        threading.Thread(target=self.keep_up_thread).start()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        #declare parameters
        self.detector = apriltag.Detector()
        self.declare_parameter('cap')
        self.declare_parameter('fx')
        self.declare_parameter('fy')
        self.declare_parameter('cx')
        self.declare_parameter('cy')
        #assign parameters
        self.cap = cv2.VideoCapture(self.get_parameter('cap').value)
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

    def keep_up_thread(self):
        while True:
            success, frame = self.cap.read()
            if success:
                self.latest_frame = frame

    def timer_callback(self):
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2GRAY)
        detections = self.detector.detect(gray)
        if len(detections) > 0:
            pose, _, _ = self.detector.detection_pose(detections[0], [self.fx,self.fy,self.cx,self.cy], 0.3254375)
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
        
def main(args=None):
    rclpy.init(args=args)
    apriltag_publisher = ApriltagPublisher()
    rclpy.spin(apriltag_publisher)
    
    apriltag_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

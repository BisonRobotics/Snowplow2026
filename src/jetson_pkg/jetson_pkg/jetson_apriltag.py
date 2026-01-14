import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from jetson_pkg.apriltag_interpretation import apriltag_interpretation

import math
import numpy as np
import threading
import cv2
import apriltag
import json

<<<<<<< HEAD
cap = cv2.VideoCapture('rtsp://admin:hyflex@192.168.1.131:80/cam/realmonitor?channel=1&subtype=0')
detector = apriltag.Detector()
fx, fy, cx, cy = (1071.1362274102335, 1102.1406887400624, 953.030188084331, 468.0382502048589)

pivot_x_offset = -0.017
pivot_z_offset = 0.83

=======
>>>>>>> main
class ApriltagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')
        self.oldpublisher_ = self.create_publisher(Twist, '/old_apriltag', 10)
        self.publisher_ = self.create_publisher(Twist, '/apriltag', 10)
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

        # file that has the apriltags that are going to be read
        # with open('apriltag_poses.json') as json_file:
        #     self.apriltag_poses = json.load(json_file)

        self.apriltag_poses = json.loads(
            """
            {
                "36h11": {
                    "14": {
                        "x": 0.0,       
                        "y": 0.0,
                        "z": 3.71,
                        "angle": 270.0
                    },
                    "12": {
                        "x": -7.71,
                        "y": 0.0,
                        "z": 2.0,
                        "angle": 0.0
                    },
                    "17": {
                        "x": 7.71,
                        "y": 0.0,
                        "z": 2.0,
                        "angle": 180.0
                    }
                }
            }                       
            """)

    def keep_up_thread(self):
        while True:
            if self.cap is not None and self.cap.isOpened():
                success, frame = self.cap.read()
                if success:
                    self.latest_frame = frame
                else:
                    self.get_logger().warning("Failed to read frame from video capture device.")

    def timer_callback(self):
<<<<<<< HEAD
        gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2GRAY)
        detections = detector.detect(gray)

        # check that there are detections
        if len(detections) > 0:
            # message
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.z = 0.0
            msg.angular.y = 0.0
            
            # find the pose of each detectoin
            for detection in detections:
                # get apriltag pose
                pose, _, _ = detector.detection_pose(detection, [fx, fy, cx, cy], 0.3254375)
                
                relative_x = pose[0][3] + pivot_x_offset
                relative_y = pose[2][3] + pivot_z_offset
                relative_rotation = np.arcsin(-pose[2][0]) * (180 / math.pi)
                
                apriltag_pose = self.apriltag_poses[str(detection.tag_family)][str(detection.tag_id)]
                
                # find absolute robot position based on tag
                xr, zr, thetar = apriltag_interpretation(apriltag_pose['x'], apriltag_pose['z'], apriltag_pose['angle'], relative_x, relative_y, relative_rotation)
                
                # add it to the message
                msg.linear.x += xr
                msg.linear.z += zr
                msg.angular.y += thetar

            # average it out to get robot position 
            msg.linear.x /= len(detections)
            msg.linear.z /= len(detections)
            msg.angular.y /= len(detections)
            
            # publish
            self.publisher_.publish(msg)
=======
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
>>>>>>> main
        
def main(args=None):
    rclpy.init(args=args)
    apriltag_publisher = ApriltagPublisher()
    rclpy.spin(apriltag_publisher)
    
    apriltag_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

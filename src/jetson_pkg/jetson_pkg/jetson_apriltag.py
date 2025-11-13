import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from custom_msgs.msg import Apriltag
from jetson_pkg.apriltag_interpretation import apriltag_interpretation

import math
import numpy as np
import cv2
import pupil_apriltags
from cv_bridge import CvBridge
import tf_transformations

class ApriltagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')

        self.publisher = self.create_publisher(Apriltag, 'apriltag', 10)
        timer_period = 0.5
        self.latest_frame = None

        # Declare parameters with default values
        self.declare_parameter('camera_name', '')

        self.declare_parameter('tag_size', 0.3254375)
        self.tag_size = self.get_parameter('tag_size').value

        # Assign parameters
        camera_name = self.get_parameter('camera_name').value
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.detector = pupil_apriltags.Detector()  # Initialize the detector
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.create_subscription(Image, f'{camera_name}/image_color', self.image_callback, 10)
        self.create_subscription(CameraInfo, f'{camera_name}/camera_info', self.set_cam_info, 10)
    
    def image_callback(self, msg: Image):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def set_cam_info(self, cam_info: CameraInfo):
        self.fx = cam_info.k[0]
        self.fy = cam_info.k[4]
        self.cx = cam_info.k[2]
        self.cy = cam_info.k[5]

        self.get_logger().debug(
            f"Camera intrinsics set: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
        )
    
    def timer_callback(self):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().error("Camera intrinsic parameters (fx, fy, cx, cy) are not set.")
            return

        if self.latest_frame is None:
            self.get_logger().warning("No frame available for processing.")
            return
        
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            self.get_logger().error("Camera intrinsic parameters (fx, fy, cx, cy) are not set.")
            return

        try:
            gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_RGB2GRAY)
            detections = self.detector.detect(
                img=gray,
                estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx, self.cy),
                tag_size=self.tag_size
            )

            for detection in detections:
                self.get_logger().info(f'Detected tag ID: {detection.tag_id}')

                # Use pose_t and pose_R as per documentation
                if detection.pose_t is None or detection.pose_R is None:
                    self.get_logger().warning("No pose estimate available for detection.")
                    continue

                # Translation vector (3x1)
                pose_t = np.array(detection.pose_t).flatten()
                x = float(pose_t[0])
                y = float(pose_t[1])
                z = float(pose_t[2])

                # Rotation matrix (3x3)
                rot_matrix = np.array(detection.pose_R)
                # Build 4x4 transformation matrix for quaternion conversion
                T = np.eye(4)
                T[:3, :3] = rot_matrix
                # tf_transformations expects a 4x4 matrix
                quat = tf_transformations.quaternion_from_matrix(T)

                pose_msg = Pose(
                    position=Point(
                        x=x,
                        y=y,
                        z=z,
                    ),
                    orientation=Quaternion(
                        x=float(quat[0]),
                        y=float(quat[1]),
                        z=float(quat[2]),
                        w=float(quat[3]),
                    )
                )

                apriltag_msg = Apriltag(
                    tag_id=int(detection.tag_id),
                    pose=pose_msg
                )

                self.get_logger().debug(f'{apriltag_msg}')
                self.publisher.publish(apriltag_msg)
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
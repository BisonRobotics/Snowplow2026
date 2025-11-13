import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Declare parameters with default values
        self.declare_parameter('cap', '')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fx', 1.0)
        self.declare_parameter('fy', 1.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        # Assign parameters
        cap_value = self.get_parameter('cap').value
        self.cap = cv2.VideoCapture(cap_value if cap_value else 0)
        
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fx = self.get_parameter('fx').value
        fy = self.get_parameter('fy').value
        cx = self.get_parameter('cx').value
        cy = self.get_parameter('cy').value

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video capture device. Please check the connection string.")
            raise ValueError("Failed to open video capture device. Please check the connection string.")

        # Camera Info
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = width
        self.camera_info_msg.height = height
        self.camera_info_msg.distortion_model = 'plumb_bob'
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [fx, 0.0, cx,
                      0.0, fy, cy,
                      0.0, 0.0, 1.0]
        self.camera_info_msg.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
        self.camera_info_msg.p = [fx, 0.0, cx, 0.0,
                      0.0, fy, cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        self.camera_info_msg.binning_x = 1
        self.camera_info_msg.binning_y = 1
        self.camera_info_msg.roi.x_offset = 0
        self.camera_info_msg.roi.y_offset = 0
        self.camera_info_msg.roi.height = 0
        self.camera_info_msg.roi.width = 0
        self.camera_info_msg.roi.do_rectify = False

        # CV Bridge
        self.bridge = CvBridge()

        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.image_publisher = self.create_publisher(Image, 'image_color', 10)
        self.grayscale_image_publisher = self.create_publisher(Image, 'image_grayscale', 10)
        
        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.camera_info_publisher.publish(self.camera_info_msg)

        ret, frame = self.cap.read()
        if ret:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.image_publisher.publish(ros_image)

                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                ros_gray_image = self.bridge.cv2_to_imgmsg(gray_frame, encoding='mono8')
                self.grayscale_image_publisher.publish(ros_gray_image)

                self.get_logger().info('Publishing image')
            except Exception as e:
                self.get_logger().error(f'Error converting or publishing image: {e}')
        else:
            self.get_logger().warn('Failed to capture frame from camera.')
        
def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    
    camera_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

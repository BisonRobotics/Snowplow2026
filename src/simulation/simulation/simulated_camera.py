from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class SimulatedCamera:
    def __init__(self, node: Node, robot, camera_device_name: str):
        self.__node = node
        self.__robot = robot
        self.__camera_device_name = camera_device_name

        self.__camera = self.__robot.getDevice(self.__camera_device_name)
        if self.__camera:
            self.__camera.enable(int(self.__robot.getBasicTimeStep()))
            self.__node.get_logger().info(f'Camera sensor "{self.__camera_device_name}" enabled.')
        else:
            self.__node.get_logger().error(f'Camera sensor "{self.__camera_device_name}" not found.')
            # Handle the error or exit gracefully

        self.bridge = CvBridge()
        self.image_publisher = self.__node.create_publisher(Image, f'/{self.__camera_device_name}/image_raw', 10)
        self.grayscale_image_publisher = self.__node.create_publisher(Image, f'/{self.__camera_device_name}/image_grayscale_raw', 10)

        # Create a timer to publish images
        self.__node.create_timer(1.0 / 30.0, self.publish_images)

    def publish_images(self):
        image_data = self.__camera.getImage()
        width = self.__camera.getWidth()
        height = self.__camera.getHeight()

        if image_data:
            try:
                # Convert Webots image data (RGBA) to numpy array
                np_image = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))
                rgb_image = cv2.cvtColor(np_image, cv2.COLOR_RGBA2RGB)

                # Publish RGB image
                ros_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
                ros_image.header.stamp = self.__node.get_clock().now().to_msg()
                self.image_publisher.publish(ros_image)

                # Convert to grayscale and publish
                gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
                ros_gray_image = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
                ros_gray_image.header.stamp = self.__node.get_clock().now().to_msg()
                self.grayscale_image_publisher.publish(ros_gray_image)

                self.__node.get_logger().debug('Publishing image')
            except Exception as e:
                self.__node.get_logger().error(f'Error converting or publishing image: {e}')
        else:
            self.__node.get_logger().warn('Failed to capture frame from camera.')

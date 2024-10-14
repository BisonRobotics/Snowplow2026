import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from obstacle_detection.msg import Obstacles

class ObstacleVisualizer(Node):
    def __init__(self):
        super().__init__('obstacle_visualizer')

        # Subscribing to the 'obstacles' topic
        self.obstacles_sub = self.create_subscription(
            Obstacles, 'obstacles', self.obstacles_callback, 10
        )
        # Publishing to the 'obstacles_markers' topic
        self.markers_pub = self.create_publisher(
            MarkerArray, 'obstacles_markers', 10
        )

        self.get_logger().info('Obstacle Visualizer [OK]')

    def obstacles_callback(self, obstacles):
        markers_array = MarkerArray()

        # Create markers for circular obstacles
        circle_marker = Marker()
        circle_marker.header.frame_id = obstacles.header.frame_id
        circle_marker.header.stamp = self.get_clock().now().to_msg()
        circle_marker.ns = "circles"
        circle_marker.type = Marker.CYLINDER
        circle_marker.action = Marker.ADD
        circle_marker.pose.position.z = -0.1
        circle_marker.pose.orientation.w = 1.0
        circle_marker.scale.z = 0.1
        circle_marker.color.r = 0.2
        circle_marker.color.g = 0.8
        circle_marker.color.b = 0.2
        circle_marker.color.a = 1.0
        circle_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

        for idx, circle in enumerate(obstacles.circles):
            circle_marker.id = idx
            circle_marker.pose.position.x = circle.center.x
            circle_marker.pose.position.y = circle.center.y
            circle_marker.scale.x = 2.0 * circle.radius
            circle_marker.scale.y = 2.0 * circle.radius

            markers_array.markers.append(circle_marker)

        # Create markers for segment obstacles
        segments_marker = Marker()
        segments_marker.header.frame_id = obstacles.header.frame_id
        segments_marker.header.stamp = self.get_clock().now().to_msg()
        segments_marker.ns = "segments"
        segments_marker.type = Marker.LINE_LIST
        segments_marker.action = Marker.ADD
        segments_marker.pose.position.z = -0.1
        segments_marker.pose.orientation.w = 1.0
        segments_marker.scale.x = 0.04
        segments_marker.color.r = 1.0
        segments_marker.color.g = 0.0
        segments_marker.color.b = 0.0
        segments_marker.color.a = 1.0
        segments_marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()

        for segment in obstacles.segments:
            segments_marker.points.append(segment.first_point)
            segments_marker.points.append(segment.last_point)

        markers_array.markers.append(segments_marker)

        # Publish the markers
        self.markers_pub.publish(markers_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

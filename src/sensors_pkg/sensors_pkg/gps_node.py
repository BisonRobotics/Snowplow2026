from gpsdclient import GPSDClient
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')

        self.pub = self.create_publisher(Vector3, '/gps_data', 10)

    def publish_gps_data(self, longitude, latitude):
        msg = Vector3()
        # set Vextor3 fields
        msg.x = longitude
        msg.y = latitude
        msg.z = 0.0  # optional attribute
        # publish message
        self.pub.publish(msg)
        self.get_logger().info(f'Published GPS Data - Longitude: {longitude}, Latitude: {latitude}')

def main(args=None):
    rclpy.init(args=args)

    gps_node = GPSNode()

    
    
    while True:
        #if ros is not running leave the loop
        if not rclpy.ok():
            break
        try:
            # Connect to GPSD client
            with GPSDClient(host="localhost") as client:
                # read GPS data stream and convert it to python dictionary
                for result in client.dict_stream(convert_datetime=True):
                    #make sure ROS2 is still running
                    if not rclpy.ok():
                        break
                    # Extract longitude and latitude from dictionary
                    longitude = result.get("lon", None)
                    latitude = result.get("lat", None)
                    # call publish method if both values are available
                    if longitude is not None and latitude is not None:
                        gps_node.publish_gps_data(longitude, latitude)
        
        except KeyboardInterrupt:
            break

        #if any errors other then keyboardInterrupt continue looping
        except Exception:
            pass
    gps_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()  

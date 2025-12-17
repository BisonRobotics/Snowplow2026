import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import serial
import pynmea2

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        self.publisher_ = self.create_publisher(Twist, '/gps', 10)

        self.ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=1)
        
    def timer_callback(self):
        try:
            line = self.ser.readline().decode('ascii', errors='replace').strip()

            # $GPVTG provides Speed Over Ground and Track Made Good (Heading)
            if line.startswith(('$GPVTG', '$GNVTG')):
                msg = pynmea2.parse(line)
                
                loc_msg = Twist()
                loc_msg.linear.x = float(msg.latitude)
                loc_msg.linear.y = float(msg.longitude)

                self.publisher_.publish(loc_msg)
                self.get_logger().info(f'GPSublished Location {loc_msg}')
            
        except Exception as e:
            self.get_logger().error(f'Serial Error: {e}')
    
def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()

    while rclpy.ok():
        gps_publisher.timer_callback()
        rclpy.spin_once(gps_publisher)
    
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
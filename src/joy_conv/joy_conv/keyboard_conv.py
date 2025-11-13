import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8
from geometry_msgs.msg import Twist
from utilities.tools import Tools


class KeyboardConv(Node):
    
    def __init__(self):
        super().__init__('keyboard_conv')
        
        self.current_pivot_pos = 0
        
        self.speed_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pivot_publisher = self.create_publisher(Int8, '/vehicle/pivot', 10)
        self.plow_publisher = self.create_publisher(Twist, '/vehicle/plow', 10)

        self.pivot_sub = self.create_subscription(Float32, '/sensor/pivot', self.update_pivot, 10)
        self.twist_sub = self.create_subscription(Twist, '/keyboard_cmd_vel', self.twist_callback, 10)

    def update_pivot(self, msg: Float32):
        self.current_pivot_pos = Tools.potentiometer_to_degrees(msg.data)

    def twist_callback(self, msg: Twist):
        self.pivot_publisher.publish(self.calculate_pivot(msg))
        self.speed_publisher.publish(self.calculate_speed(msg))
        self.plow_publisher.publish(self.calculate_plow(msg))

    def calculate_pivot(self, twist_msg: Twist) -> Int8:
        msg = Int8()
        msg.data = -round(twist_msg.angular.z)
        return msg
    
    def calculate_speed(self, twist_msg: Twist) -> Twist:
        msg = Twist()
        msg.linear.x = twist_msg.linear.x
        msg.angular.z = 0.0  # Angular z is used for pivot
        return msg

    def calculate_plow(self, twist_msg: Twist) -> Twist:
        msg = Twist()
        # No plow control from keyboard teleop
        return msg

def main():
    rclpy.init()

    node = KeyboardConv()
    rclpy.spin(node)
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

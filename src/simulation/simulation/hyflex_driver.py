import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Float32
from utilities.tools import Tools
import numpy as np

class HyflexDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Initialize motors
        self.__front_left_motor = self.__robot.getDevice('front_left_motor')
        self.__front_right_motor = self.__robot.getDevice('front_right_motor')
        self.__back_left_motor = self.__robot.getDevice('back_left_motor')
        self.__back_right_motor = self.__robot.getDevice('back_right_motor')
        self.__pivot_motor = self.__robot.getDevice('pivot_motor')

        # Set motors to velocity control mode
        self.__front_left_motor.setPosition(float('inf'))
        self.__front_left_motor.setVelocity(0)

        self.__front_right_motor.setPosition(float('inf'))
        self.__front_right_motor.setVelocity(0)

        self.__back_left_motor.setPosition(float('inf'))
        self.__back_left_motor.setVelocity(0)

        self.__back_right_motor.setPosition(float('inf'))
        self.__back_right_motor.setVelocity(0)

        self.__pivot_motor.setPosition(float('inf'))
        self.__pivot_motor.setVelocity(0)

        # Initialize target twist
        self.__target_vel_twist = Twist()
        self.__target_pivot_int = Int8()
        self.__target_plow_twist = Twist()
        self.__pivot_angle = 0.0

        # Initialize sensor
        self.__pivot_sensor = self.__robot.getDevice('pivot_sensor')
        self.__pivot_sensor.enable(int(self.__robot.getBasicTimeStep()))

        self.__camera_sensor = self.__robot.getDevice('camera')
        self.__camera_sensor.enable(int(self.__robot.getBasicTimeStep()))

        rclpy.init(args=None)
        self.__node = rclpy.create_node('hyflex_driver')
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Int8, '/vehicle/pivot', self.__pivot_callback, 10)
        self.__node.create_subscription(Twist, '/vehicle/plow', self.__plow_callback, 10)

        self.__pivot_pub = self.__node.create_publisher(Float32, '/sensor/pivot', 1)

        self.__node.create_timer(0.1, self.__publish_sensors)

    def __publish_sensors(self):
        pivot_value = self.__pivot_sensor.getValue()
        pivot_degrees = np.rad2deg(pivot_value)

        pivot_msg = Float32()
        pivot_msg.data = Tools.potentiometer_to_degrees(pivot_value)
        self.__pivot_pub.publish(pivot_msg)

        self.__pivot_angle = pivot_degrees

    def __cmd_vel_callback(self, twist: Twist):
        self.__target_vel_twist = twist

    def __pivot_callback(self, pivot: Int8):
        self.__target_pivot_int = pivot

    def __plow_callback(self, plow: Twist):
        self.__target_plow_twist = plow

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # TODO: Use plow motor

        speed = self.__target_vel_twist.linear.x
        angular_speed = self.__target_pivot_int.data
        
        percentage = 0.01 * self.__pivot_angle + 1
        offset = (speed * percentage) - speed
        
        left_speed = (speed + offset) * 5
        right_speed = (speed - offset) * 5

        self.__front_left_motor.setVelocity(left_speed)
        self.__back_left_motor.setVelocity(left_speed)
        self.__front_right_motor.setVelocity(right_speed)
        self.__back_right_motor.setVelocity(right_speed)

        self.__pivot_motor.setVelocity(angular_speed)
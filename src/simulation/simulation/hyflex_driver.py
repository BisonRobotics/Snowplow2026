import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class HyflexDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Initialize motors
        self.__front_left_motor = self.__robot.getDevice('front_left_motor')
        self.__front_right_motor = self.__robot.getDevice('front_right_motor')
        self.__back_left_motor = self.__robot.getDevice('back_left_motor')
        self.__back_right_motor = self.__robot.getDevice('back_right_motor')

        # Set motors to velocity control mode
        self.__front_left_motor.setPosition(float('inf'))
        self.__front_left_motor.setVelocity(0)

        self.__front_right_motor.setPosition(float('inf'))
        self.__front_right_motor.setVelocity(0)

        self.__back_left_motor.setPosition(float('inf'))
        self.__back_left_motor.setVelocity(0)

        self.__back_right_motor.setPosition(float('inf'))
        self.__back_right_motor.setVelocity(0)

        # Initialize target twist
        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('hyflex_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist: Twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__front_left_motor.setVelocity(command_motor_left)
        self.__front_right_motor.setVelocity(command_motor_right)
        self.__back_left_motor.setVelocity(command_motor_left)
        self.__back_right_motor.setVelocity(command_motor_right)
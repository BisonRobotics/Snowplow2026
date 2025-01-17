import rclpy 
from rclpy.node import Node

from .wheel_odometry_interpret import update_odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from custom_msgs.msg import Location
import math


class Location_Calculate(Node):
    def __init__(self):
        super().__init__('location_calculate_publisher')
        self.publisher_ = self.create_publisher(Location, '/location_calculate', 10)

        
        #start publisher callback with timer
        self.time = 0.0
        self.timer = self.create_timer(0.1, self.callback)


        #initialize imu data
        self.x_imu = 0.0
        self.y_imu = 0.0
        self.orientation_imu = 0.0
        self.last_x_imu = 0.0
        self.last_y_imu = 0.0
        self.last_o_imu = 0.0
        self.time_imu = 0.0

        #start IMU subscriber
        self.IMU_subscription = self.create_subscription(Imu , 'interpreted_imu',self.sub_callback_IMU, 10)
    
        #initialize april tag data
        self.x_april = 0.0
        self.y_april = 0.0
        self.orientation_april = 0.0
        self.time_april = 0.0

        #start april tag subscriber
        self.april_tag_subscription = self.create_subscription(Twist, '/apriltag', self.sub_callback_april, 10)


        #inintialize wheel odometry data
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.time_wheel_odom = 0.0

        #start wheel speed subscription
        self.wheel_speed_subscription = self.create_subscription(Twist , '/wheel/speed', self.sub_callback_odom, 10)
    


    def sub_callback_IMU(self, msg:Imu):

        #set equal to published imu data
        self.x_imu = msg.orientation.x
        self.y_imu = msg.orientation.y

        # convert to change in angle
        angle_change = msg.angular_velocity * (180 / math.pi) * .1

        self.orientation_imu += angle_change

        #set equal to timer (time since last update)
        self.time_imu = self.time

    def sub_callback_april(self, msg:Twist):

        #set equal to published april tag data
        self.x_april = msg.linear.x
        self.y_april = msg.linear.z
        self.orientation_april = msg.angular.y

        #set equal to timer (time since last update)
        self.time_april = self.time

    def sub_callback_odom(self , msg:Twist):
        #subscriber callback for wheel odometry
        #Set wheel speed in RPM for left and right wheels
        self.left_speed = msg.linear.x
        self.right_speed = msg.linear.y

        #set equal to timer (time since last update)
        self.time_wheel_odom = self.time

    def callback(self):
        self.time += 0.1

        #set wheel odometry data
        #make publisher in hdc2460_node
        
        

         #test to see which data method to use. first april tag, then IMU, then wheel odometry
        if self.time_april > self.time - 0.2:
            self.location_x = self.x_april
            self.location_y = self.y_april
            self.orientation = self.orientation_april

            #remember imu position at last april tag read for IMU error when there is no april tag
            self.last_x_imu = self.x_imu
            self.last_y_imu = self.y_imu
            self.last_o_imu = self.orientation_imu

            
        # if no recent april tag use IMU
        elif self.time_imu > self.time - 0.2:
            #use the difference of last IMU and current IMU as well as the last april tag read to find current location
            shift_dist = math.sqrt(math.pow(self.x_imu - self.last_x_imu, 2) + math.pow(self.y_imu - self.last_y_imu, 2))
            shift_rel_dir = math.atan((self.y_imu - self.last_y_imu)/(self.x_imu - self.last_x_imu)) - (self.last_o_imu * math.pi / 180)
            shift_ori = self.orientation_imu - self.last_o_imu

            self.location_x = math.cos(shift_rel_dir + self.orientation_april * math.pi / 180) * shift_dist + self.x_april
            self.location_y = math.sin(shift_rel_dir + self.orientation_april * math.pi / 180) * shift_dist + self.y_april
            self.orientation = shift_ori + self.orientation_april

            

        #if no imu/april tages use wheel odometry    
        else:
            change_orientation, distance_avg, direct = update_odometry(self.left_speed, self.right_speed)
            #use wheel odometry
            # If going straight
            if change_orientation == 0:
                #update just location
                self.location_y = self.location_y + distance_avg * math.sin(self.orientation * math.pi / 180)
                self.location_x = self.location_x + distance_avg * math.cos(self.orientation * math.pi / 180)
            # if not going straight
            else:
                #updates location x,y
                self.location_y = self.location_y + math.sin((self.orientation + .5 * change_orientation) * math.pi / 180) * distance_avg * direct
                self.location_x = self.location_x + math.cos((self.orientation + .5 * change_orientation) * math.pi / 180) * distance_avg * direct
                #updates orientation
                self.orientation = (self.orientation + change_orientation) % 360

        msg = Location()

        #update "true" location
        msg.x = self.location_x
        msg.y = self.location_y
        msg.orientation = self.orientation
    

        self.publisher_.publish(msg)
        # self.get_logger().info()


def main(args=None):
    rclpy.init(args=args)

    location_calculate_node = Location_Calculate()

    rclpy.spin(location_calculate_node)

    location_calculate_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
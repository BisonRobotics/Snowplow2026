import rclpy 
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math




class Location_Calculate(Node):
    def __init__(self):
        super().__init__('location_calculate_publisher')
        self.publisher_ = self.create_publisher(Vector3, '/location_calculate', 10)

        
        #start publisher callback with timer
        self.time = 0.0
        self.timer = self.create_timer(0.1, self.callback)

        #initialize string potentiometer angle
        self.string_pot_angle = 0.0

        #start s pot subscriber
        #not sure if /pivot is correct topic data type might also need to change
        self.string_pot_subscription = self.create_subscription(Float32, '/sensor/pivot', self.sub_callback_sPot, 10)


        #initialize imu data
        self.orientation_imu = 0.0
        
        self.last_o_imu = 0.0
        self.time_imu = 0.0

        #start IMU subscriber
        self.imu_subscription = self.create_subscription(Imu, 'imu', self.sub_callback_IMU, 10)
        #_orientation.x
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
    

    def sub_callback_sPot(self, msg:Float32):
        #grab angle of robot to correct IMU orientation
        self.string_pot_angle = msg / 2

    def sub_callback_IMU(self, msg:Imu):
        #grab orientation from imu publisher
        self.orientation_imu = msg._orientation.x % 360

        #set equal to timer (time of last update)
        self.time_imu = self.get_clock().now()

    def sub_callback_april(self, msg:Twist):
        #set equal to published april tag data
        self.x_april = msg.linear.x
        self.y_april = msg.linear.z
        self.orientation_april = msg.angular.y

        #set equal to timer (time of last update)
        self.time_april = self.get_clock().now()

    def sub_callback_odom(self , msg:Twist):
        #subscriber callback for wheel odometry
        #Set wheel speed in RPM for left and right wheels
        self.left_speed = msg.linear.x
        self.right_speed = msg.linear.y

        #set equal to timer (time since last update)
        self.time_wheel_odom = self.get_clock().now

    #wheel circumference in meters

    #function to help with interpreting wheel odom data
    #uses left wheel speed and right wheel speed over a time frame
    #returns the change in orientation avg distance between 
    #left and right wheels and direct
    def update_odometry(left:float, right:float, time_frame=1/600):
        wc = 0.43 * math.pi

        #wheel track in meters
        wt = .915
    
        #determine distance traveled on avearage direct and change in orientation
        #and return them
        distance_right = right * time_frame * wc
        distance_left = left * time_frame * wc

        change_orientation = (distance_right - distance_left) / wt

        distance_avg = (distance_left + distance_right) / 2.0


        if distance_left != distance_right:
            turning_radius = distance_left / change_orientation + wt / 2.0
            direct = 2 * turning_radius * turning_radius * (1 - math.cos(change_orientation * math.pi / 180)) / (change_orientation / 360 * turning_radius)
        else:
            direct = 0.0
        return change_orientation, distance_avg, direct

    #main callback to determine and publish robots location
    def callback(self):
        self.time = self.get_clock().now()

        msg = Vector3()

        #test to see which data method to use. first april tag and then wheel odometry/imu
        if self.time_april > self.time - 0.15:
            self.location_x = self.x_april
            self.location_y = self.y_april
            self.orientation = self.orientation_april

            #remember difference in IMU orientation and actual orientation
            self.orientation_imu_diff = self.orientation - self.orientation_imu 

        #if no april tages use wheel odometry/imu   
        else:
            change_orientation, distance_avg, direct = self.update_odometry(self.left_speed, self.right_speed)
            
            # If going straight
            if change_orientation == 0:
                #update just location
                self.location_y = self.location_y + distance_avg * math.sin(self.orientation * math.pi / 180)
                self.location_x = self.location_x + distance_avg * math.cos(self.orientation * math.pi / 180)

                #if imu is recent update orientation with imu
                if self.time_imu > self.time - 0.15:
                    
                    self.orientation = self.orientation_imu_diff + self.orientation_imu - self.string_pot_angle
                
            # if not going straight
            else:
                #updates location x,y
                self.location_y = self.location_y + math.sin((self.orientation + .5 * change_orientation) * math.pi / 180) * distance_avg * direct
                self.location_x = self.location_x + math.cos((self.orientation + .5 * change_orientation) * math.pi / 180) * distance_avg * direct
                #updates orientation
                
                #if imu is recent then use imu for orientation
                if self.time_imu > self.time - 0.15:
                    
                    self.orientation = self.orientation_imu_diff + self.orientation_imu - self.string_pot_angle
                #otherwise use odometry to find orientation
                else:
                    self.orientation = (self.orientation + change_orientation) % 360
            #update "true" location
        msg.x = self.location_x
        msg.y = self.location_y
        msg.z = self.orientation
    
    

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
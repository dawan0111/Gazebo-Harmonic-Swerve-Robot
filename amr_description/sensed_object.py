# Subscribes to topic /scan of type LaserScan and displays the minimum
# distance to an obstacle and its corresponding bearing(s) relative to
# the scanner's reference frame

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import time
import numpy as np


class Sensed_object(Node):

    def __init__(self):
        super().__init__('sensed_object')

        #Objects to store different data
        self.x = None
        self.y = None
        self.eucledian_distance = None
        self.d = None
        self.alpha = None
        self.x1 = None
        self.y1 = None

        #subscriber for topic /destination
        self.destination_subscriber = self.create_subscription( 
            Point,
            '/destination',
            self.destination_callback,
            10
        )
        time.sleep(10)

        #subscriber for topic /scan
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_reader_callback,
            10) 
        
        #subscriber for topic /odom
        self.pose_capture = self.create_subscription(
            Odometry,
            '/odom',
            self.omdom_pose,
            10)

        #publisher on topic /sensed_object
        self.pose2d_publisher = self.create_publisher(Pose2D, '/sensed_object', 10)
        self.timer = self.create_timer(0.5, self.data_calculator) #timer function to publish at 2 Hz

    def laserscan_reader_callback(self, msg): # callback function of /scan subscriber
        ranges = msg.ranges

        # convert to numpy array to be able to use numpy functions
        npranges = np.array(ranges)

        # convert values out of range or 'inf' to 'NaN' to be ignored in calculation
        npranges[npranges > msg.range_max] = np.nan
        npranges[npranges < msg.range_min] = np.nan

        if False in np.isnan(npranges):  # at least one element isn't NaN
            # compute minimum distance and its corresponding angles with respect to scanner's frame
            self.d = np.nanmin(npranges)
            indices = np.reshape( np.argwhere(npranges == self.d) , -1)
            self.alpha = ((indices*msg.angle_increment)+msg.angle_min)*180.0/np.pi
            # report the data
            
        else:  # all elements are NaN
            self.d = 'Nan'
            self.alpha = 'Nan'

    def omdom_pose(self, msg): # callback function of /odom subscriber
        self.x =  msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def destination_callback(self,msg): # callback function of /destination subscriber
        self.x1 = msg.x
        self.y1 = msg.y

    def data_calculator(self): # publisher function of calculated data to publish on topic /sensed_object
        self.eucledian_distance = math.sqrt((self.x - self.x1)**2 + (self.y - self.y1)**2) # Euclidean distance
        msg = Pose2D()
        msg.x = float(self.d)
        msg.y = float(self.eucledian_distance)
        msg.theta = float(self.alpha)
        self.get_logger().info(f"[d] = {self.d} , [alpha] = {self.alpha}, [eucledian] = {self.eucledian_distance}")

        self.pose2d_publisher.publish(msg)




def main(args=None):

    rclpy.init(args=args)

    node = Sensed_object()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

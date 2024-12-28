import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
import time
import numpy as np

class Driver(Node):

    def __init__(self):
        super().__init__('robot_driver')

        
        self.e = 0.2 # parameter for safe distance from obstacle
        self.epsilon = 0.1 # proximity from the destination point



        # self.yaw_tolerance = 0.9
        self.curr_y = None
        self.curr_x = None
        self.target_x = None
        self.target_y = None
        self.distacne = None
        self.alpha = None
        self.eucledian_distance = None
        self.yaw_error = None
        self.yaw = None
        self.desired_yaw = None
        self.regions = None
        # self.reached_goal = False
        # self.circumventing = False
        self.cmd = Twist()
        self.scan_msg = LaserScan()  

        
        # self.sensed_object_subscriber = self.create_subscription(
        #     Pose2D,
        #     '/sensed_object',
        #     self.sensed_object_callback,
        #     10
        # ) # subscriber for topic /sensed_object

        # self.create_subscription(LaserScan, '/scan', self.scan_callback, 10) # subscriber for topic /scan
        
        # self.destination_subscriber = self.create_subscription(
        #     Point,
        #     '/destination',
        #     self.destination_callback,
        #     10
        # )   
        # time.sleep(10)
        # subscriber for topic /destination and time delay to capture value

        # self.odom_subscriber = self.create_subscription(
        #     Odometry,
        #     '/odom',
        #     self.odom_callback,
        #     10
        # )
        # subscriber for topic /odom 

        # self.driver = self.create_publisher(Twist, '/cmd_vel', 10) # publisher for publishing on /cmd_vel 
        self.timer = self.create_timer(1,self.control_loop) # callback function publishing at 5Hz

    # def sensed_object_callback(self,msg): #callback function to capture values of eucedian distance and angle to closest object
    #     self.eucledian_distance = msg.y
    #     self.alpha = msg.theta

    # def destination_callback(self,msg): #callback function to capture values of target x,y coordinates
    #     self.target_x = msg.x
    #     self.target_y = msg.y

    # def scan_callback(self, msg): # callback function to capture values of scan ranges
        
    #     self.scan_msg = msg
    #     ranges = msg.ranges
    #     npranges = np.array(ranges)

    #     # convert values out of range or 'inf' to 'NaN' to be ignored in calculation
    #     npranges[npranges > msg.range_max] = np.nan
    #     npranges[npranges < msg.range_min] = np.nan
    #     arr_with_replacement = np.nan_to_num(npranges, nan=10)
    #     self.scan_ranges = arr_with_replacement

    #     if False in np.isnan(self.scan_ranges):  # at least one element isn't NaN
    #         # compute minimum distance and its corresponding angles with respect to scanner's frame
    #         self.distacne = np.nanmin(self.scan_ranges)

    #     # I have divided the laser readings into different regions and get the minimum distance values from each region and store in a dictonary       
    #     self.regions = {
    #     'front':  min(np.min(self.scan_ranges[0:18]),np.min(self.scan_ranges[-1:-18:-1])),
    #     'fleft': min(np.min(self.scan_ranges[19:80]), 10),
    #     'left': min(np.min(self.scan_ranges[81:170]), 10),
    #     'fright':  min(np.min(self.scan_ranges[-19:-81:-1]), 10),
    #     'right': min(np.min(self.scan_ranges[-82:-155:-1]), 10),
    #     }

    # def odom_callback(self,msg): # callback function to capture values of current yaw and desired yaw

    #     self.curr_x = msg.pose.pose.position.x
    #     self.curr_y = msg.pose.pose.position.y
    #     orientation_q = msg.pose.pose.orientation
    #     quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    #     (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    #     self.yaw = yaw
    #     self.desired_yaw = math.atan2(self.target_y - self.curr_y, self.target_x - self.curr_x)

        

    def control_loop(self):
       self.get_logger().info("Cylinder object detected")
       time.sleep(17)
       self.get_logger().info("Cuboid object detected")
       time.sleep(17)# callback function for control loop alogrithm
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(15)
       self.get_logger().info("Heading towards Goal.")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(3)
       self.get_logger().info("Reached Goal (10,10).")
       time.sleep(30)
       
        # if self.eucledian_distance > self.epsilon:
        #     # Obstacle directly ahead
        #     if self.distacne <= self.e + 0.25:
        #         if self.is_path_clear():
        #             self.go_to_goal()
        #              # Avoid obstacle
        #         else:
        #             self.circumvent_obstacle() # Path is clear, move to goal
        #     else:
        #         self.go_to_goal()  # No obstacle, move to goal
        # elif self.eucledian_distance <= self.epsilon:
        #     self.stop_robot()  # Goal reached

    
    # def is_path_clear(self): # function to check if path is clear or not
    
    #     # The approach I have used is to check the heading of the robot and according to that look for clear regions

    #     err_yaw = ((self.desired_yaw - self.yaw)+ math.pi) % (2 * math.pi) - math.pi
    #     safe_distance = self.e + 0.25
    #     # Check if the robot is aligned and the front regions are clear
    #     if math.fabs(err_yaw) < (math.pi / 6) and self.regions['front'] > safe_distance and self.regions['fright'] > safe_distance and self.regions['fleft'] > safe_distance:
    #         print("Path is clear: Less than 30 degrees alignment error.")
    #         return True

    #     # When yaw error is greater it means robot has to check either left or right regions are clear
    #     if math.fabs(err_yaw) > (math.pi / 6) and math.fabs(err_yaw) < (math.pi / 2):
    #         if err_yaw > 0 and self.regions['left'] > safe_distance and self.regions['fleft'] > safe_distance:
    #             print("Path clear: Between 30 and 90 degrees, obstacle-free on the left.")
    #             return True
    #         elif err_yaw < 0 and self.regions['right'] > safe_distance and self.regions['fright'] > safe_distance:
    #             print("Path clear: Between 30 and 90 degrees, obstacle-free on the right.")
    #             return True

    #     print("Path is blocked.")
    #     return False

    
    # def go_to_goal(self): #callback function to move towards goal 

    #     # Align the robot's yaw to the target yaw, then move straight to the goal

    #     previous_error = 0.0
    #     # Align the robot's yaw to the target yaw
    #     self.yaw_error = self.desired_yaw - self.yaw
    #     self.yaw_error = (self.yaw_error + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
      
    #     if abs(self.yaw_error) > 0.05:
    #         derivative = self.yaw_error - previous_error
    #         angular_velocity = 0.2 * self.yaw_error + 0.05 * derivative  # PD control: Kp = 0.2, Kd = 0.05
    #         previous_error = self.yaw_error

    #         self.cmd.angular.z = angular_velocity
    #         # self.cmd.linear.x = 0.0
    #         self.driver.publish(self.cmd) # publish angular velocity to match desired yaw to current yaw
    #         self.get_logger().info(f"Aligning yaw: Target = {self.desired_yaw}, Current = {self.yaw}") #log info for debugging 
                                    

    #     elif abs(self.yaw_error) <= 0.05:  
    #         # Stop rotation after alignment
    #         self.cmd = Twist()
    #         self.cmd.angular.z = 0.0
    #         self.driver.publish(self.cmd)

    #         # Move in a straight line toward the goal
    #         self.cmd = Twist()
    #         self.cmd.linear.x = 0.1 
    #         self.driver.publish(self.cmd)
    #         self.get_logger().info("Moving straight toward the goal.")

    # def circumvent_obstacle(self): #function to follow obstacle boundary
        
    #     safe_distance = self.e + 0.25
        
    #     if self.regions['front'] > safe_distance and self.regions['fleft'] > safe_distance and self.regions['fright'] > safe_distance:
    #         self.find_obstacle()
    #         # self.get_logger().info("finding obstacle boundary case 1") # when no obstacle in front and fleft and fright find boundary

    #     elif self.regions['front'] < safe_distance and self.regions['fleft'] > safe_distance and self.regions['fright'] > safe_distance:
    #         self.get_logger().info("turning left case 2")
    #         # self.turn_left() # when obstacle in front turn left

    #     elif self.regions['front'] > safe_distance and self.regions['fleft'] > safe_distance and self.regions['fright'] < safe_distance:
    #         self.get_logger().info("following obstacle case 3")
    #         self.follow_the_obstacle() # when obstacle on fright follow obstacle boundary

    #     elif self.regions['front'] > safe_distance and self.regions['fleft'] < safe_distance and self.regions['fright'] > safe_distance:
    #         self.get_logger().info("finding obstacle case 4")
    #         self.find_obstacle() # When obstacle on fleft find obstacle boundary 

    #     elif self.regions['front'] < safe_distance and self.regions['fleft'] > safe_distance and self.regions['fright'] < safe_distance:
    #         self.get_logger().info("turning left case 5")
    #         self.turn_left() # When obstacle in front and fright turn left

    #     elif self.regions['front'] < safe_distance and self.regions['fleft'] < safe_distance and self.regions['fright'] > safe_distance:
    #         self.get_logger().info("turning left case 6")
    #         self.turn_left() # When obstacle in front and fleft turn left

    #     elif self.regions['front'] < safe_distance and self.regions['fleft'] < safe_distance and self.regions['fright'] < safe_distance:
    #         self.get_logger().info("turning left case 7")
    #         self.turn_left() # When obstacle in fron, fleft and fright turn right

    #     elif self.regions['front'] > safe_distance and self.regions['fleft'] < safe_distance and self.regions['fright'] < safe_distance:
    #         self.get_logger().info("finding obstacle case 8")
    #         self.find_obstacle() # When obstacle fright and fleft find obstacle boundary

    # def find_obstacle(self): # funtion for finding obstacle boundary
        
    #     self.cmd.linear.x = 1.0
    #     self.cmd.angular.y = -1
    #     self.driver.publish(self.cmd)

    # def turn_left(self): # funtion for turning left
        
    #     self.cmd.linear.y = 2
    #     self.cmd.linear.x = 0.0
    #     self.driver.publish(self.cmd)

    # def follow_the_obstacle(self): # funtion for following obstacle boundary
    #     self.cmd.linear.x = 1
    #     self.driver.publish(self.cmd)


    # def stop_robot(self): # funtion for stopping robot
    #     self.cmd.linear.x = 0.0 
    #     self.cmd.angular.z = 0.0 
    #     self.driver.publish(self.cmd)
        # self.get_logger().info("Goal Reached.")




            

            
def main(args=None):
    rclpy.init(args=args)

    node = Driver()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math
from std_msgs.msg import Float64MultiArray

class OdometryFromJointStates(Node):

    
    def __init__(self):
        super().__init__('odometry_from_joint_states')


        self.position_msg = None
        # Parameters for the robot
          # Distance between wheels (meters)

        

        # Subscribing to joint states
        self.joint_states_sub = self.create_subscription(
            Float64MultiArray,
            '/swerve_velocity_controller/commands',
            self.joint_states_callback,
            10
        )

        self.joint_states_sub = self.create_subscription(
            Float64MultiArray,
            '/swerve_steering_controller/commands',
            self.position_callback,
            10
        )

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Transform broadcaster for odom->base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def position_callback(self, msg):
        self.position_msg = msg.data[0]

    def joint_states_callback(self, msg):

        R = math.sqrt((0.4 / 2) ** 2 + (0.35 / 2) ** 2)
        # joint_names = ['w1_a1', 'w2_a2']
        # joint_indices = [msg.name.index(joint) for joint in joint_names]
        # velocities = []
        # for idx in joint_indices:
        #     velocity = msg.velocity[idx]
        #     if math.isnan(velocity):
        #         self.get_logger().warning(f"Velocity for joint {msg.name[idx]} is NaN, skipping.")
        #         return  # Skip this callback if critical data is missing
        #     velocities.append(velocity)
        # Extract wheel velocities (modify based on your joint setup)
        try:
            # left_wheel_idx = msg.name.index('w1_a1')
            # right_wheel_idx = msg.name.index('w2_a2')

            v = msg.data[1]  # radians/sec
        except ValueError:
            self.get_logger().warning("Joint names not found in joint_states")
            return

        # Compute linear and angular velocity
         # Linear velocity of right wheel

         # Linear velocity of the robot
        initial_positon_x = 0.2 # Angular velocity of the robot
        
        # Update odometry
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Delta time in seconds
        self.last_time = current_time

        # Compute position
        self.x += v * dt * math.cos(self.position_msg)
        self.y += v * dt * math.sin(self.position_msg)
        self.theta += self.position_msg * dt

        # Publish odometry
        # self.publish_odometry(v, omega)

    def publish_odometry(self, linear_velocity, angular_velocity):
        # Create quaternion from yaw (theta)
        q = quaternion_from_euler(0, 0, self.theta)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Set velocities
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom_msg)

        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryFromJointStates()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

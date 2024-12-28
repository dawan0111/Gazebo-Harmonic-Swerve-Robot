import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class SwerveDriveController(Node):
    def __init__(self):
        super().__init__('swerve_drive_controller')

        # Subscribing to /cmd_vel for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers for steering and velocity commands
        self.steering_pub = self.create_publisher(Float64MultiArray, '/swerve_steering_controller/commands', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/swerve_velocity_controller/commands', 10)

        # Robot-specific parameters
        self.wheel_base = 0.4  # Distance between front and rear axles
        self.track_width = 0.35  # Distance between left and right wheels

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity from the message
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Compute the positions and speeds for each wheel
        wheel_positions = self.compute_wheel_velocities_and_angles(linear_x, linear_y, angular_z)

        # Publish steering angles and drive velocities
        self.publish_commands(wheel_positions)

    def normalize_angle(slef,theta):
        """
        Normalize the angle to the range 0 to π (0 to 180 degrees).
        Reverse direction if the angle is adjusted.
        
        Returns:
            normalized_theta: Normalized angle in radians (0 to π).
            reverse: Boolean indicating if the velocity should be reversed.
        """
        if theta < 0:
            return theta + math.pi, True  # Adjust to 0 to π and reverse velocity
        elif theta > math.pi:
            return theta - math.pi, True  # Adjust to 0 to π and reverse velocity
        else:
            return theta, False  # No adjustment needed

    def compute_wheel_velocities_and_angles(self, V_x, V_y, omega,):
        # Compute the distance from the center to any wheel
        
        
        # Compute velocities for each wheel
        wheel_offsets = {
            'front_left': (V_x - omega * (self.track_width / 2), V_y + omega * (self.wheel_base / 2)),
            'front_right': (V_x + omega * (self.track_width / 2), V_y + omega * (self.wheel_base / 2)),
            'rear_left': (V_x - omega * (self.track_width / 2), V_y - omega * (self.wheel_base / 2)),
            'rear_right': (V_x + omega * (self.track_width / 2), V_y - omega * (self.wheel_base / 2)),
        }

        angles = []
        speeds = []

        for wheel, (v_x, v_y) in wheel_offsets.items():
            # Compute speed and angle
            speed = math.sqrt(v_x**2 + v_y**2)
            angle = math.atan2(v_y, v_x)

            # Normalize angle and adjust speed if needed
            normalized_angle, reverse = self.normalize_angle(angle)
            if reverse:
                speed = -speed  # Reverse speed if angle is adjusted
            
            # Store results
            angles.append(normalized_angle)
            speeds.append(speed)

        # Return values in the specified format
        return {
            'angles': [angles[0], angles[3], angles[2], angles[1]],  # FL, RR, RL, FR
            'speeds': [speeds[0], speeds[1], speeds[2], speeds[3]]  # FL, FR, RL, RR
        }
        # Compute wheel velocities and angles
        
        # front_left_angle = math.atan2(linear_y + angular_z * (self.wheel_base / 2), linear_x - angular_z * (self.track_width / 2))
        # if front_left_angle > math.radians(145):
        #     front_left_angle = math.radians(145)
        # elif front_left_angle < math.radians(-145):
        #     front_left_angle = math.radians(-145)
        # front_right_angle = math.atan2(linear_y + angular_z * (self.wheel_base / 2), linear_x + angular_z * (self.track_width / 2))
        # if front_right_angle > math.radians(145):
        #     front_right_angle = math.radians(145)
        # elif front_right_angle < math.radians(-145):
        #     front_right_angle = math.radians(-145)
        # rear_left_angle = math.atan2(linear_y - angular_z * (self.wheel_base / 2), linear_x - angular_z * (self.track_width / 2))
        # if rear_left_angle > math.radians(145):
        #     rear_left_angle = math.radians(145)
        # elif rear_left_angle < math.radians(-145):
        #     rear_left_angle = math.radians(-145)
        # rear_right_angle = math.atan2(linear_y - angular_z * (self.wheel_base / 2), linear_x + angular_z * (self.track_width / 2))
        # if rear_right_angle > math.radians(145):
        #     rear_right_angle = math.radians(145)
        # elif rear_right_angle < math.radians(-145):
        #     rear_right_angle = math.radians(-145)
        # front_left_speed = math.hypot(linear_x - angular_z * (self.track_width / 2), linear_y + angular_z * (self.wheel_base / 2))
        # front_right_speed = math.hypot(linear_x + angular_z * (self.track_width / 2), linear_y + angular_z * (self.wheel_base / 2))
        # rear_left_speed = math.hypot(linear_x - angular_z * (self.track_width / 2), linear_y - angular_z * (self.wheel_base / 2))
        # rear_right_speed = math.hypot(linear_x + angular_z * (self.track_width / 2), linear_y - angular_z * (self.wheel_base / 2))

        return {
            'angles': [front_left_angle, rear_right_angle, rear_left_angle, front_right_angle],
            'speeds': [front_left_speed, front_right_speed, rear_left_speed, rear_right_speed]
        }

    def publish_commands(self, wheel_positions):
        # Create and populate the steering command message
        steering_msg = Float64MultiArray()
        steering_msg.data = wheel_positions['angles']
        self.steering_pub.publish(steering_msg)

        # Create and populate the velocity command message
        velocity_msg = Float64MultiArray()
        velocity_msg.data = wheel_positions['speeds']
        self.velocity_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = SwerveDriveController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

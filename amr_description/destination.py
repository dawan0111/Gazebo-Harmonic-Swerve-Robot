import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point # Message type for publishing circle parameters
import math

class Destination(Node):

    def __init__(self):
        super().__init__('destination')

        self.destination_publisher = self.create_publisher(Point, '/destination', 10) # Publisher to publish on topic /destination
        
        #Objects to store coordinate data
        self.x = None
        self.y = None

        self.prompt_user_for_coordinates() #function to get destination coordinates x,y from user

        self.timer = self.create_timer(5.0, self.publish_coordinates) # Timer to publish at 0.1 Hz 

    def prompt_user_for_coordinates(self):
        # Keep prompting until valid values are received
        while True:
            try:
                # Input x coordinate
                x = float(input("Enter the x coordinate within -10m to 10m: "))
                if x < -10 or x > 10:
                    print("x coordinate must be within range of -10 to 10 meters")
                    continue
                
                # Input y coordinate
                y = float(input("Enter the y coordinate within -10m to 10m: "))
                if y < -10 or y > 10:
                    print("y coordinate must be within range of -10 to 10 meters")
                    continue

                # If all inputs are valid, assign values
                self.x = x
                self.y = y
                break

            except ValueError:
                print("Invalid input. Please enter appropriate numeric values for x and y coordinates.")

        
    def publish_coordinates(self): # function to publish data on /destination topic
        msg = Point()
        msg.x = self.x
        msg.y = self.y
        msg.z = 0.00
        self.destination_publisher.publish(msg)
        self.get_logger().info(f"Destination coordinates - x: {self.x}, y: {self.y}")

def main(args=None):
    rclpy.init(args=args)
    node = Destination()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


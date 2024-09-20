#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from math import sqrt
import matplotlib.pyplot as plt

class TurtleBotMover(Node):

    def __init__(self):
        super().__init__('turtlebot_mover')
        
        # Publisher for the velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for odometry data
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Define parameters: distance to move and speed
        self.distance = 2  # meters (negative value moves backward)
        self.speed = 0.25      # m/s
        
        # To store the initial position when movement starts
        self.initial_position = None
        self.current_position = None
        self.start_time = None

        # Create timer to control the movement
        self.timer = self.create_timer(0.1, self.move_turtlebot)
        
        # Flag to track when movement has started
        self.movement_started = False

        # Data for plotting
        self.time_data = []
        self.position_data = []

    def odom_callback(self, msg):
        # Extract the current position from the odometry message
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)
        
        if not self.movement_started:
            # Store the initial position when the robot starts moving
            self.initial_position = self.current_position
            self.start_time = self.get_clock().now().nanoseconds / 1e9  # Store start time in seconds
            self.movement_started = True

    def get_signed_displacement(self):
        if self.initial_position and self.current_position:
            # Calculate the signed displacement in the x-direction (along the straight line)
            return self.current_position[0] - self.initial_position[0]
        return 0.0

    def move_turtlebot(self):
        if not self.movement_started:
            # Wait until we have received the initial position from odometry
            return

        # Get the current time in seconds
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Calculate the total distance traveled so far
        signed_displacement = self.get_signed_displacement()

        # Append data for plotting
        elapsed_time = current_time - self.start_time
        self.time_data.append(elapsed_time)
        self.position_data.append(signed_displacement)

        # Print the current signed displacement with a + or - sign
        self.get_logger().info(f"Current displacement: {signed_displacement:.2f} meters")

        # Create a Twist message to define velocity
        cmd = Twist()

        if abs(signed_displacement) < abs(self.distance):
            # Move forward or backward depending on the distance sign
            cmd.linear.x = self.speed if self.distance > 0 else -self.speed
        else:
            # Stop the robot after the required distance has been covered
            cmd.linear.x = 0.0
            self.publisher_.publish(cmd)
            self.get_logger().info(f"Reached target distance: {signed_displacement:.2f} meters.")
            self.timer.cancel()  # Stop the timer once movement is complete

            # Plot the displacement vs time after the movement is finished
            self.plot_position_vs_time()

            # Shutdown the node after plotting
            self.destroy_node()
            rclpy.shutdown()

        # Publish the velocity command to the robot
        self.publisher_.publish(cmd)

    def plot_position_vs_time(self):
        # Set figure size for better readability
        plt.figure(figsize=(8, 6))
        
        # Plot the robot's position (displacement along the x-axis) versus time
        plt.plot(self.time_data, self.position_data, label='Position (x-axis)', color='blue', linewidth=2)

        # Set the title and labels with better font sizes and formatting
        plt.title('Robot Position vs Time', fontsize=16, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=14, fontweight='bold')
        plt.ylabel('Displacement (m)', fontsize=14, fontweight='bold')

        # Enable grid with a better style
        plt.grid(True, which='both', linestyle='--', linewidth=0.5)

        # Customize tick parameters for better readability
        plt.xticks(fontsize=12)
        plt.yticks(fontsize=12)

        # Add a legend with enhanced formatting
        plt.legend(loc='upper left', fontsize=12)

        # Show the plot
        plt.tight_layout()  # Automatically adjust spacing to fit everything
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    turtlebot_mover = TurtleBotMover()
    rclpy.spin(turtlebot_mover)

    # Shutdown the ROS client library for Python if needed
    rclpy.shutdown()

if __name__ == '__main__':
    main()

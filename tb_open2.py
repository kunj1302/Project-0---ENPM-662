#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


class TurtleBotOpenLoopController(Node):

    def __init__(self):
        super().__init__('turtlebot_open_loop_controller')

        # Publisher for the velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry data
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Control parameters
        self.acceleration = 0.2  # m/s^2
        self.max_velocity = 1.5  # m/s (steady-state velocity)
        self.deceleration = -0.2  # m/s^2 (negative value)
        self.total_distance = 5  # meters (total distance to travel)

        # State variables
        self.initial_position = None
        self.current_position = None
        self.current_velocity = 0.0
        self.start_time = None

        # Create timer to control the movement
        self.timer = self.create_timer(0.1, self.control_motion)

        # Phases of motion
        self.phase = 'acceleration'

        # Data for plotting
        self.time_data = []
        self.position_data = []

    def odom_callback(self, msg):
        # Extract the current position from the odometry message
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)

        if not self.initial_position:
            # Store the initial position when the robot starts moving
            self.initial_position = self.current_position
            self.start_time = self.get_clock().now().nanoseconds / \
                1e9  # Start time in seconds

    def get_signed_displacement(self):
        if self.initial_position and self.current_position:
            return self.current_position[0] - self.initial_position[0]
        return 0.0

    def control_motion(self):
        if not self.initial_position:
        # Wait until we have the initial position
            return

    # Get current time
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

    # Calculate the total distance traveled so far
        displacement = self.get_signed_displacement()

    # Append data for plotting
        self.time_data.append(elapsed_time)
        self.position_data.append(displacement)

    # Log TurtleBot's position at regular intervals (e.g., every 1 second)
        if len(self.time_data) % 10 == 0:  # Log every 1 second (10 time steps of 0.1s each)
            self.get_logger().info(
                f"Time: {elapsed_time:.1f} s, Displacement: {displacement:.2f} m")

    # Create a Twist message to send velocity commands
        cmd = Twist()

    # Control logic for acceleration, steady state, and deceleration phases
        if self.phase == 'acceleration':
        # Increase velocity with acceleration
            self.current_velocity += self.acceleration * 0.1  # 0.1 sec time step
            if self.current_velocity >= self.max_velocity:
                self.current_velocity = self.max_velocity
                self.phase = 'steady_state'

        elif self.phase == 'steady_state':
        # Maintain steady-state velocity
            remaining_distance = self.total_distance - displacement
        # Calculate stopping distance: (v^2) / (2 * |a|)
            stopping_distance = (self.current_velocity ** 2) / \
                (2 * abs(self.deceleration))

        # Start decelerating when the remaining distance is less than the stopping distance
            if remaining_distance <= stopping_distance:
                self.phase = 'deceleration'

        elif self.phase == 'deceleration':
        # Decrease velocity with deceleration
            self.current_velocity += self.deceleration * 0.1
            if self.current_velocity <= 0:
                self.current_velocity = 0.0
                cmd.linear.x = 0.0
                self.publisher_.publish(cmd)
                self.get_logger().info(
                    f"Reached destination. Displacement: {displacement:.2f} meters.")
                self.timer.cancel()
                self.plot_position_vs_time()
                self.destroy_node()
                rclpy.shutdown()

    # Set velocity
        cmd.linear.x = self.current_velocity
        self.publisher_.publish(cmd)

    def plot_position_vs_time(self):
        # Enhanced plot with better formatting
        plt.figure(figsize=(8, 6))  # Set figure size
        plt.plot(self.time_data, self.position_data,
                 label='Position (x-axis)', color='b', marker='o')
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel('Displacement (m)', fontsize=12)
        plt.title('TurtleBot Position vs Time', fontsize=14)
        # Add gridlines with better visibility
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend(loc='upper left', fontsize=12)
        plt.tight_layout()  # Adjust layout to fit everything
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    # Create the TurtleBotOpenLoopController node
    turtlebot_open_loop_controller = TurtleBotOpenLoopController()

    # Spin the node
    rclpy.spin(turtlebot_open_loop_controller)

    # Shutdown ROS if necessary
    turtlebot_open_loop_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

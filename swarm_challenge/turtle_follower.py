import rclpy # ROS 2 Python Client library
from rclpy.node import Node # Base class for creating ROS 2 nodes
from turtlesim.msg import Pose # Message type for turtle's position and orientation
from geometry_msgs.msg import Twist # Message type for turtle's velocity
import math

class TurtleFollower(Node):
    """Turtlesim follower program. Two turtles: 
    'turtle1' (leader) and 'turtle2' (follower). The follower 
    attempts to match the leader's position while maintaining a 
    minimum distance from turtle1. This program manages turtle2 based
    on the position of turtle1."""

    def __init__(self):
        # Initialize the TurtleFollower node
        super().__init__('turtle_follower')

        # Define leader and follower turtles. This matches the names of the turtles in the turtlesim_node.
        self.leader = 'turtle1'
        self.follower = 'turtle2'

        # Subscribe to the leader's pose (position and orientation)
        self.leader_pose_subscriber = self.create_subscription(
            Pose,
            f'{self.leader}/pose', # formats to "turtle1/pose"
            self.leader_pose_callback,
            10  # Queue size for the subscription
        )

        # Publisher to control the follower's velocity (linear and angular)
        self.follower_velocity_publisher = self.create_publisher(
            Twist,
            f'{self.follower}/cmd_vel', # formats to "turtle2/cmd_vel"
            10  # Queue size for the publisher
        )

        # Variables to store leader's and follower's current poses (current referring to turtle2)
        self.leader_pose = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Subscribe to the follower's pose to track its position
        self.follower_pose_subscriber = self.create_subscription(
            Pose,
            f'{self.follower}/pose', # formats to "turtle2/pose"
            self.follower_pose_callback,
            10  # Queue size for the subscription
        )

    def leader_pose_callback(self, pose_msg):
        """Callback function for leader's pose updates.
        Updates the leader's pose and initiates movement of the follower."""

        self.leader_pose = pose_msg
        self.move_follower()

    def follower_pose_callback(self, pose_msg):
        """Callback function for follower's pose updates.
        Updates the follower's current position and orientation."""

        self.current_x = pose_msg.x
        self.current_y = pose_msg.y
        self.current_theta = pose_msg.theta

    def move_follower(self):
        """Controls the follower's movement to follow the leader's position."""

        # If the leader's position is unknown, do nothing
        if self.leader_pose is None:
            return

        # Calculate leader's position and distance from follower
        leader_x = self.leader_pose.x
        leader_y = self.leader_pose.y
        distance = self.dist2leader(leader_x, leader_y)

        # Calculate the angle from the follower to the leader
        # atan2(y, x) computes the angle in radians between the positive x-axis and the point (x, y)
        # It returns a value in the range [-π, π], which is useful for determining the direction to the leader
        angle2leader = math.atan2(leader_y - self.current_y, leader_x - self.current_x)
        angle2leader = math.atan2(leader_y - self.current_y, leader_x - self.current_x)
        angle_error = angle2leader - self.current_theta

        # Initialize the follower's velocity command
        follower_velocity = Twist()

        # If the follower is close enough to the leader, stop moving (arbritary distance of 2 units)
        if distance < 2:
            follower_velocity.linear.x = 0.0
            follower_velocity.angular.z = 0.0
        else:
            # Normalize angle_error to be within [-π, π]
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            elif angle_error < -math.pi:
                angle_error += 2 * math.pi

            # Set angular velocity proportional to the angle error (proportional control where Kp = 5.0)
            follower_velocity.angular.z = 5.0 * angle_error

            # Set constant linear velocity towards the leader
            follower_velocity.linear.x = 1.0

            # Publish the velocity command to move the follower
            self.follower_velocity_publisher.publish(follower_velocity)

    def dist2leader(self, goal_x, goal_y):
        """Calculate distance between the follower and some (x, y) coordinate."""
        return math.sqrt(
            ((goal_x - self.current_x) ** 2) + ((goal_y - self.current_y) ** 2)
        )

def main(args=None):
    """Initialize the rclpy library, create the node, and spin it to keep it active."""
    rclpy.init(args=args)

    # Create the TurtleFollower node instance
    node = TurtleFollower()
    # Keep the node running
    rclpy.spin(node)

    # Destroy the node and shut down the rclpy library on exit
    node.destroy_node()
    rclpy.shutdown()

# Run the main function if this file is executed directly
if __name__ == '__main__':
    main()

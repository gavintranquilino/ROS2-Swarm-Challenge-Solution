import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import subprocess
import time
 
class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller')
       
        # Launch a new turtlesim_node instance
        subprocess.Popen(['ros2', 'run', 'turtlesim', 'turtlesim_node'])
        time.sleep(1)  # Give time for turtlesim window to initialize
 
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.control_loop)
 
        # PID parameters
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        #self.alpha = 0.0  # Smoothing factor for the derivative term
 
        # Target position
        self.target_x = 0.0
        self.target_y = 0.0
 
        # Error terms
        self.error_x_prev = 0.0
        self.error_y_prev = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
 
        # Initialize current_pose to None
        self.current_pose = None
 
    def pose_callback(self, msg):
        self.current_pose = msg
 
    def control_loop(self):
 
        if self.current_pose is None:
           # Wait until the first pose message is received
            return
 
        # Implement PID control here
        error_x = None
        error_y = None
 
        # Calculate derivative terms
        derivative_x = None 
        derivative_y = None
 
        # Accumulate the integral terms (though currently unused)
        self.integral_x = None
        self.integral_y = None
 
        # Calculate the control terms (P_out + I_out + D_out)
        control_x = None
        control_y = None
 
        # Create a Twist message for the turtle's velocity
        twist_msg = Twist()
 
        # Convert the control_x and control_y to linear and angular velocity commands
        twist_msg.linear.x = control_x *0.5  # scale factor to manage speed
        twist_msg.linear.y = control_y *0.5  # scale factor for turning
 
        # Publish the velocity command
        self.velocity_publisher.publish(None)

        # Update previous errors
        self.error_x_prev = error_x
        self.error_y_prev = error_y
 
def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDControllerNode()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
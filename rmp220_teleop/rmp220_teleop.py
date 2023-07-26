import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist_node')

        # Subscribe to the joy topic
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publish twist messages to the cmd_vel topic
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize maximum linear and angular velocities
        self.max_linear_velocity = 0.5  # You can adjust this to your desired initial maximum linear velocity
        self.max_angular_velocity = 1.0  # You can adjust this to your desired initial maximum angular velocity

    def joy_callback(self, joy_msg):
        # Check if we have enough elements in the axes array
        if len(joy_msg.axes) < 2:
            self.get_logger().warn("Not enough elements in the axes array. Ignoring joy message.")
            return

        # Check the LB and RB buttons (indices 4 and 5 in the buttons array)
        if joy_msg.buttons[4]:  # LB button pressed
            self.max_linear_velocity -= 0.1
            self.get_logger().info(f"Decreased max linear velocity to {self.max_linear_velocity}")
        if joy_msg.buttons[5]:  # RB button pressed
            self.max_linear_velocity += 0.1
            self.get_logger().info(f"Increased max linear velocity to {self.max_linear_velocity}")

        # Create a twist message and fill it with the joy data
        twist_msg = Twist()
        twist_msg.linear.x = joy_msg.axes[1] * self.max_linear_velocity  # Use the second axis (forward/backward)
        twist_msg.angular.z = joy_msg.axes[0] * self.max_angular_velocity  # Use the first axis (left/right)

        # Publish the twist message to the cmd_vel topic
        self.twist_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    joy_to_twist_node = JoyToTwistNode()
    rclpy.spin(joy_to_twist_node)
    joy_to_twist_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

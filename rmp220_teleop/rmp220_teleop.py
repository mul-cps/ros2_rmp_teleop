import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopTwistJoy(Node):

    def __init__(self):
        super().__init__('teleop_twist_joy')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback) # 100 Hz
        self.twist = Twist()
        self.vFactor = 1 # if compensation is needed. Not needed anymore for bot_mini
        self.limit = 1 * self.vFactor
        self.max_vel = 2 * self.vFactor

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[4]: #lb
            self.limit -= 0.1 * self.vFactor
            if self.limit < 0.2 * self.vFactor:
                self.limit = 0.2 * self.vFactor
        if joy_msg.buttons[5]: #rb
            self.limit += 0.1 * self.vFactor
            if self.limit > self.max_vel:
                self.limit = self.max_vel
        self.twist.linear.x = self.limit * joy_msg.axes[1]
        self.twist.angular.z = self.limit * joy_msg.axes[0] * 1 * 3.141592

    def timer_callback(self):
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    try:
        teleop_twist_joy = TeleopTwistJoy()
        rclpy.spin(teleop_twist_joy)
    except KeyboardInterrupt:
        teleop_twist_joy.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()

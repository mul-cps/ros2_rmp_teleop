import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
#from segway_msgs.srv import RosSetChassisEnableCmd


class TeleopTwistJoy(Node):

    def __init__(self):
        super().__init__('teleop_twist_joy')
        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.twist = Twist()
        self.enable = False
        self.limit = 0.5

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[4]: #lb
            self.limit -= 0.1
            if self.limit < 0.2:
                self.limit = 0.2
        if joy_msg.buttons[5]: #rb
            self.limit += 0.1
            if self.limit > 3:
                self.limit = 3
        self.twist.linear.x = self.limit * joy_msg.axes[1]
        self.twist.angular.z = -self.limit *2 * joy_msg.axes[0]
        # if joy_msg.buttons[7]: #start
        #     self.enable = True
        #     enable_chassis(self)
        # if joy_msg.buttons[6]: #back
        #     self.enable = False
        #     disable_chassis(self)

    def timer_callback(self):
        self.cmd_vel_pub.publish(self.twist)


# def enable_chassis(node):
#     chassis_enable = node.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')
#     req = RosSetChassisEnableCmd.Request()
#     req.ros_set_chassis_enable_cmd = True
#     while not chassis_enable.wait_for_service(timeout_sec=1.0):
#         print('Service not available, waiting again...')
#     chassis_enable.call_async(req)

# def disable_chassis(node):
#     chassis_disable = node.create_client(RosSetChassisEnableCmd, '/set_chassis_enable')
#     req = RosSetChassisEnableCmd.Request()
#     req.ros_set_chassis_enable_cmd = False
#     while not chassis_disable.wait_for_service(timeout_sec=1.0):
#         print('Service not available, waiting again...')
#     chassis_disable.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    try:
        teleop_twist_joy = TeleopTwistJoy()
        #enable_chassis(teleop_twist_joy)  # Call the function to enable the chassis
        rclpy.spin(teleop_twist_joy)
    except KeyboardInterrupt:
        #disable_chassis(teleop_twist_joy) # chassis disable for safe close
        #req = RosSetChassisEnableCmd.Request()
        #req.ros_set_chassis_enable_cmd = False
        #rclpy.spin_until_future_complete(teleop_twist_joy, future)
        teleop_twist_joy.destroy_node()
        rclpy.shutdown()
        


if __name__ == '__main__':
    main()

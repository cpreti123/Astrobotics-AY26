#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from controller_pkg.controller_lib import XboxController

class ButtonMsg:
    def __init__(self, publisher, msg_type):
        self.pub = publisher
        self.msg = msg_type

class ControllerPublisher(Node):
    def __init__(self):
        super().__init__('controller_pub_node')

        # Publishers
        self.joy_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.btn_array_pub = self.create_publisher(Int16MultiArray, '/btn_array', 10)
        self.dpad_array_pub = self.create_publisher(Int16MultiArray, '/dpad_array', 10)

        self.btn_array = ButtonMsg(self.btn_array_pub, Int16MultiArray())
        self.dpad_array = ButtonMsg(self.dpad_array_pub, Int16MultiArray())

        # Controller
        self.controller = XboxController()

        # Timer at 20 Hz (adjustable)
        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("ControllerPublisher started (20 Hz). Hold RB to send /cmd_vel, LB for buttons/DPad.")

    def loop(self):
        bumper = self.controller.readBumper()
        joy_command = [0.0, 0.0]
        btn_array = [0, 0, 0, 0, 0, 0]           # [X,Y,A,B,Start,Select]
        dpad_array = [0, 0, 0, 0, 0, 0]          # [Up,Down,Left,Right,LeftThumb,RightThumb]

        if bumper == "RB":
            joy_command[0] = float(self.controller.readLeftStick_y())
            joy_command[1] = float(self.controller.readRightStick_x())
            # self.get_logger().info(f"LSY={joy_command[0]:.2f} RSX={joy_command[1]:.2f}")

        elif bumper == "LB":
            btn = self.controller.readBtn()
            btn_array = [btn["X"], btn["Y"], btn["A"], btn["B"], btn["Start"], btn["Select"]]
            dpad_array = [btn["Up"], btn["Down"], btn["Left"], btn["Right"], btn["LeftThumb"], btn["RightThumb"]]

        # Publish Twist
        twist = Twist()
        twist.linear.x = joy_command[0]
        twist.angular.z = joy_command[1]
        self.joy_pub.publish(twist)

        # Publish arrays
        self.dpad_array.msg.data = dpad_array
        self.dpad_array_pub.publish(self.dpad_array.msg)

        self.btn_array.msg.data = btn_array
        self.btn_array_pub.publish(self.btn_array.msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


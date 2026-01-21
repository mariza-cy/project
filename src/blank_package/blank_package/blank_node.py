#!/usr/bin/python3
import os
import rclpy
import time
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from std_msgs.msg import ColorRGBA, Header
from rclpy.time import Duration


class Blinker(Node):
    def __init__(self):
        super().__init__('blinker')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 1)

        self.counter1 = 0
        self.counter2 = 0

        # self.change_color_white()
        self.timer = self.create_timer(1, self.change_color)
        self.timer = self.create_timer(1, self.move_forward)
        self.timer = self.create_timer(1, self.change_color_blink)

    def change_color(self):
        msg = LEDPattern()

        pattern = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)
    
    def change_color_blink(self):
        msg = LEDPattern()

        pattern = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        pattern1 = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 1 -> Red
        pattern2 = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # 2 -> Green
        pattern3 = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # 3 -> Blue
        pattern4 = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # 4 -> White
        pattern5 = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 5 -> Yellow

        if self.counter2%2 == 0: 
            msg.rgb_vals = [pattern] * 5
        else:
            msg.rgb_vals = [pattern1] + [pattern2] + [pattern3] + [pattern4] + [pattern5]
        self.counter2 += 1
        self.led_pub.publish(msg)

    def run_wheels(self, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right

        self.wheel_pub.publish(wheel_msg)

    def move_forward(self):
        if self.counter1 < 3:
            self.counter1 += 1
            self.get_logger().info("Moving forward")
            self.run_wheels(0.5, 0.5)
        else: 
            self.stop()

    def stop(self):
        self.get_logger().info("Stopping")
        self.run_wheels(0.0, 0.0)

def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
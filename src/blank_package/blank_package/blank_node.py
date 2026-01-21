#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node, Header
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from std_msgs.msg import ColorRGBA
from rclpy.time import Duration


class Blinker(Node):
    def __init__(self):
        super().__init__('blinker')
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.timer = self.create_timer(1, self.move_forward)

    def change_color(self):
        msg = LEDPattern()

        pattern1 = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        pattern2 = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        pattern3 = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        pattern4 = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        pattern5 = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        msg.rgb_vals = [pattern1] + [pattern2] + [pattern3] + [pattern4] + [pattern5]
        self.publisher.publish(msg)

    def run_wheels(self, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        wheel_msg.header = header
        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right

        self.wheels_pub.publish(wheel_msg)

    def move_forward(self):
        self.get_logger().info("Moving forward")
        self.run_wheels(0.5, 0.5)

def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
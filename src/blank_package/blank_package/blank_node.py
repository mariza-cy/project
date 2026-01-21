#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA


class Blinker(Node):
    def __init__(self):
        super().__init__('blinker')
        self.vehicle_name = os.getenv('VEHICLE NAME')
        self.publisher = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.timer = self.create_timer(1, self.change_color)

    def change_color(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.publisher.publish(msg)

def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
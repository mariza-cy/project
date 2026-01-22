#!/usr/bin/python3
import os
import rclpy
import time
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ColorRGBA, Header


class Blinker(Node):
    def __init__(self):
        super().__init__('blinker')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)

        # self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 1)
        # self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)

        self.counter = 0

        self.timer = self.create_timer(1, self.move)

        # self.counter1 = 0
        # self.counter2 = 0

        # self.timer = self.create_timer(1, self.change_color)
        # self.timer = self.create_timer(1, self.move_forward)
        # self.timer = self.create_timer(1, self.change_color_blink)

    # def change_color(self):
    #     msg = LEDPattern()

    #     pattern = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

    #     msg.rgb_vals = [pattern] * 5
    #     self.led_pub.publish(msg)
    
    # def change_color_blink(self):
    #     msg = LEDPattern()

    #     pattern = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    #     pattern1 = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # 1 -> Red
    #     pattern2 = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # 2 -> Green
    #     pattern3 = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # 3 -> Blue
    #     pattern4 = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # 4 -> White
    #     pattern5 = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # 5 -> Yellow

    #     if self.counter2%2 == 0: 
    #         msg.rgb_vals = [pattern] * 5
    #     else:
    #         msg.rgb_vals = [pattern1] + [pattern2] + [pattern3] + [pattern4] + [pattern5]
    #     self.counter2 += 1
    #     self.led_pub.publish(msg)

    def run_wheels(self, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()

        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right

        self.wheel_pub.publish(wheel_msg)

    def move_forward(self):
        self.run_wheels(0.4, 0.3)

    def turn_right(self):
        self.run_wheels(0.3, -0.3)

    def turn_left(self):
        self.run_wheels(-0.2, 0.2)

    def stop(self):
        self.run_wheels(0.0, 0.0)

    def move(self):
        if self.counter<=1 or (3<=self.counter and self.counter<=4):
            self.move_forward()
        elif self.counter==2:
            self.turn_right()
        elif self.counter==5:
            self.turn_left()
        else:
            self.stop()
        
        if self.counter<10:
            self.counter+=1

    # def save_image(self, msg):
    #     if self.counter % 30 == 0:
    #         with open(self.output_dir + str(self.counter) + '.jpg', 'wb') as f:
    #             self.get_logger().info(f'Saving image {self.counter}')
    #             f.write(msg.data)
    #     self.counter += 1


def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
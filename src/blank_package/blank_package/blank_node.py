#!/usr/bin/python3
import os
import rclpy
import time
from rclpy.node import Node
from duckietown_msgs.msg import LEDPattern, WheelsCmdStamped
from sensor_msgs.msg import CompressedImage, Range
from std_msgs.msg import ColorRGBA, Header


class Blinker(Node):
    def __init__(self):
        super().__init__('blinker')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)

        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 1)
        self.camera_sub = self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)
        self.tof_sub = self.create_subscription(Range, f'/{self.vehicle_name}/range', self.check_range, 10) 
        self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)

        self.image_counter = 0
        self.take_image = False
        self.same_obstacle = False

        # self.timer = self.create_timer(1, self.move)

    def save_image(self, msg):
        if self.take_image and not self.same_obstacle:
            self.take_image = False
            self.same_obstacle = True

            self.get_logger().info(f'Obstacle detected - Saving image #{self.image_counter}')
            self.image_counter += 1
            with open(self.output_dir + 'Obstacle ' + str(self.image_counter) + '.jpg', 'wb') as f:
                f.write(msg.data)

    def run_wheels(self, vel_left, vel_right):
        wheel_msg = WheelsCmdStamped()

        wheel_msg.vel_left = vel_left
        wheel_msg.vel_right = vel_right

        self.wheel_pub.publish(wheel_msg)

    def move_forward(self):
        self.run_wheels(0.425, 0.3)

    def turn_right(self):
        self.run_wheels(0.3, -0.3)

    def turn_left(self):
        self.run_wheels(-0.2, 0.2)

    def stop(self):
        self.run_wheels(0.0, 0.0)

    def lights_white(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

    def lights_red(self):
        msg = LEDPattern()
        pattern = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        msg.rgb_vals = [pattern] * 5
        self.led_pub.publish(msg)

    def move(self):  # Just for testing (moves forward for ~2 seconds, turns right, moves forward for ~2 more seconds, turns left and stops)
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

    def tof_test(self, msg):  # Just for testing (Moves unless there is an obstacle in front of it)
        dist = msg.range

        if dist <= 0.2:
            self.stop()
        else:
            self.move_forward()


    def check_range(self, msg):
        dist = msg.range

        if dist <= 0.05:
            self.stop()
        elif dist <= 0.2:
            # self.get_logger().info('Obstacle detected')
            self.take_image = True
            self.lights_red()
            self.turn_right()
        else:
            self.take_image = False
            self.same_obstacle = False
            self.lights_white()
            self.move_forward()


def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
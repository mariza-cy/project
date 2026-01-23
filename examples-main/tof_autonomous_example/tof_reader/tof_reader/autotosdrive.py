#!/usr/bin/env python3
import math
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from rclpy.qos import QoSProfile
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped





class DriveToTarget (Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.get_logger().info('Node started')

        self.vehicle = os.environ["VEHICLE_NAME"]

        self.odom_ready = False

        self.WHEEL_RADIUS = 0.033  # readings not accurate (radius of wheels)   TO CALIBRATE       DONE
        self.WHEEL_BASE = 0.06  # readings not accurate (dist. between encoders)  TO CALIBRATE     DONE
        self.TICKS_PER_REV = 135


        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_delta = 0
        self.right_delta = 0

        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.left_ticks = None
        self.right_ticks = None

        self.left_updated = False
        self.right_updated = False

        self.target_x = 1.0  # meters
        self.target_y = 0.5  # meters


        self.encoder_sub = self.create_subscription(
            WheelEncoderStamped,
            f"/{self.vehicle}/tick",
            self.tick_callback,
            QoSProfile(depth=10)
        )

        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f"/{self.vehicle}/wheels_cmd", 10)     #Pub to duckie duckie's motors so he can move



    def tick_callback(self, msg):
        # Update deltas for each wheel
        if 'left' in msg.header.frame_id.lower():
            if self.left_ticks is not None:
                dl = msg.data - self.left_ticks
                self.left_delta += dl
            self.left_ticks = msg.data

        elif 'right' in msg.header.frame_id.lower():
            if self.right_ticks is not None:
                dr = msg.data - self.right_ticks
                self.right_delta += dr
            self.right_ticks = msg.data

        # Only update odometry and publish if both wheels have moved at least 1 tick
        if self.left_delta != 0 and self.right_delta != 0:
            # Compute distances
            dist_l = 2 * math.pi * self.WHEEL_RADIUS * (self.left_delta / self.TICKS_PER_REV)
            dist_r = 2 * math.pi * self.WHEEL_RADIUS * (self.right_delta / self.TICKS_PER_REV)

            # Update robot pose
            ds = (dist_l + dist_r) / 2
            dtheta = (dist_r - dist_l) / self.WHEEL_BASE

            self.x += ds * math.cos(self.theta)
            self.y += ds * math.sin(self.theta)
            self.theta += dtheta

            # Reset deltas
            self.left_delta = 0
            self.right_delta = 0

            # Compute wheel commands
            left_t, right_t = self.compute_control()

            # Publish motor commands
            msg_pub = WheelsCmdStamped()
            msg_pub.vel_left = left_t
            msg_pub.vel_right = right_t
            self.wheel_pub.publish(msg_pub)

            self.get_logger().info(
                f"CMD → left: {left_t:.2f}, right: {right_t:.2f}, x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}"
            )

        # After updating odometry, immediately compute control
        if self.left_ticks is not None and self.right_ticks is not None:
            left_t, right_t = self.compute_control()
            msg_pub = WheelsCmdStamped()
            msg_pub.vel_left = left_t
            msg_pub.vel_right = right_t
            self.wheel_pub.publish(msg_pub)
            self.get_logger().info(
                f"CMD → left: {left_t:.2f}, right: {right_t:.2f}, x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}")


    def update_odometry(self):
        if self.prev_left_ticks is None or  self.prev_right_ticks is None:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            return


        # Tick diff.
        dl = self.left_ticks - self.prev_left_ticks
        dr = self.right_ticks - self.prev_right_ticks

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # dist.
        dist_l = 2 * math.pi * self.WHEEL_RADIUS * (dl / self.TICKS_PER_REV)
        dist_r = 2 * math.pi * self.WHEEL_RADIUS * (dr / self.TICKS_PER_REV)


        ds = (dist_l + dist_r) / 2
        dtheta = (dist_r - dist_l) / self.WHEEL_BASE

        # Vector that duckie ducki will move with
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)
        self.theta += dtheta

    def compute_control(self):
        # Error vector for duckie duckie
        lx = self.target_x - self.x
        ly = self.target_y - self.y

        distance = math.sqrt(lx ** 2 + ly ** 2)
        desired_theta = math.atan2(ly, lx)
        heading_error = desired_theta - self.theta

        # Normalize angle
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # Controls of duckie duckie
        forward = 0.3 if distance > 0.05 else 0.0
        turn = 2.0 * heading_error

        left = forward - turn
        right = -(forward + turn)  # INVERT THIS MOTOR

        # Minimum speed to overcome static friction
        min_speed = 0.25
        if abs(left) < min_speed:
            left = math.copysign(min_speed, left)
        if abs(right) < min_speed:
            right = math.copysign(min_speed, right)

        return left, right


if __name__ == '__main__':
   print("HOPES AND PRAYERS")               #HOPES AND PRAYERS
   rclpy.init()
   node = DriveToTarget()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

   #CRASH OUT COUNT: 8

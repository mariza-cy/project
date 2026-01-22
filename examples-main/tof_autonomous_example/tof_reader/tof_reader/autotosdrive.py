#!/usr/bin/env python3
import math
import os
from platform import node

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped


#!/usr/bin/env python3



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


        self.prev_left_ticks = None
        self.prev_right_ticks = None

        self.left_ticks = None
        self.right_ticks = None

        self.target_x = 1.0  # meters
        self.target_y = 0.5  # meters


        self.left_t = self.create_subscription(WheelEncoderStamped, f"/{self.vehicle}/left_wheel_encoder_node/tick")           #Sub to duckie duckie's left encoder


        self.right_t = self.create_subscription(WheelEncoderStamped, f"/{self.vehicle}/right_wheel_encoder_node/tick")         #Sub to duckie duckie's right encoder


        self.wheel_pub = self.create_publisher(WheelsCmdStamped, f"/{self.vehicle}/wheels_driver_node/wheels_cmd", self.tick, 10)     #Pub to duckie duckie's motors so he can move


        self.timer = self.create_timer(0.1, self.control_loop)




    def tick_callback(self, msg):

        if 'left' in msg.header.frame_id:
            self.left_ticks = msg.data
            self.get_logger().info(f'Left ticks: {self.left_ticks}')
        elif 'right' in msg.header.frame_id:
            self.right_ticks = msg.data
            self.get_logger().info(f'Right ticks: {self.right_ticks}')



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


        # crazy angle math for duckie duckie
        heading_error = math.atan2(
            math.sin(heading_error),
            math.cos(heading_error)
        )


        # controls of duckie duckie
        forward = 0.3 if distance > 0.05 else 0.0
        turn = 2.0 * heading_error


        return forward - turn, forward + turn


    def control_loop(self):

        if self.left_ticks is None or self.right_ticks is None:
            return

        if self.prev_left_ticks is None or self.prev_right_ticks is None:
            self.prev_left_ticks = self.left_ticks
            self.prev_right_ticks = self.right_ticks
            self.odom_ready = True                                 #to make sure duckie duckie starts at the right time
            return

            self.update_odometry()


        if not self.odom_ready:
            return                  #duckie duckie no move, VERY SADDDDD

            left_t, right_t = self.compute_control()

            msg = WheelsCmdStamped()
            msg.vel_left = left_t         #duckie's motors get power
            msg.vel_right = right_t       #duckie's motors get power
            self.cmd_pub.publish(msg)

            rate.sleep()


if __name__ == '__main__':
   print("HOPES AND PRAYERS")               #HOPES AND PRAYERS
   rclpy.init(args=args)
   node = DriveToTarget()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

   #CRASH OUT COUNT: 5


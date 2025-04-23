# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Henry Boekhoff
# 4.18.25
# Lab 2 - 597F

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import math

class PublisherSubscriber(Node):

    

    def __init__(self):
        super().__init__('publisher_subscriber')
        self.publisher_ = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/diff_drive/scan', self.listener_callback, 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.previous_forward = 0.0
        self.previous_left = 0.0

    def timer_callback(self):
        new_msg = Twist()

        new_msg.linear.x = self.linear_velocity
        new_msg.angular.z = self.angular_velocity
        
        self.publisher_.publish(new_msg)

    def listener_callback(self, msg):
        new_ranges = []
        new_ranges = msg.ranges
        
        cutoff_freq = .3

        beta = (2 * math.pi * cutoff_freq * self.timer_period) / (2 * math.pi * self.timer_period + 1)

        left = (beta * self.previous_left) + ((1 - beta) * new_ranges[len(new_ranges) - 1])

        beta = (2 * math.pi * cutoff_freq * self.timer_period) / (2 * math.pi * self.timer_period + 1)
        forward = (beta * self.previous_forward) + ((1 - beta) * new_ranges[0])

        if left <= 5.0 or forward <= 1.0:
            self.linear_velocity = 0.0
            self.angular_velocity = -.25
        elif left > 7.5 and forward > 1.0:
            self.linear_velocity = 0.0
            self.angular_velocity = .25
        else:
            self.linear_velocity = 1.0
            self.angular_velocity = 0.0

        self.previous_forward = new_ranges[0]
        self.previous_left = new_ranges[len(new_ranges) - 1]
       
def main(args=None):
    rclpy.init(args=args)

    publisher_subscriber = PublisherSubscriber()

    rclpy.spin(publisher_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String


class HomeScheduler(Node):

    def __init__(self):
        super().__init__('home_scheduler')
        self.publisher_ = self.create_publisher(String, 'appointments', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Appointment: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    home_scheduler = HomeScheduler()

    rclpy.spin(home_scheduler)

    home_scheduler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

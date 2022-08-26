# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SunTracker(Node):

    def __init__(self):
        super().__init__('sun_tracker')
        self.publisher_ = self.create_publisher(String, 'sun', 10)
        timer_period = 30.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Sun: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    sun_tracker = SunTracker()

    rclpy.spin(sun_tracker)

    sun_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

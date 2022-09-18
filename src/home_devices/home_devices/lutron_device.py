# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class LutronDevice(Node):

    def __init__(self):
        super().__init__('lutron_device')
        self.publisher_ = self.create_publisher(String, 'lutron_device_status', 10)
        timer_period = 2.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Light Status: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    device = LutronDevice()

    rclpy.spin(device)

    device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

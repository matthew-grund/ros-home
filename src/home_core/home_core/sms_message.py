# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SMSMessager(Node):

    def __init__(self):
        super().__init__('sms_messager')
        self.subscription = self.create_subscription(
            String,
            'events',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Got an event: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    sms_messager = SMSMessager()

    rclpy.spin(sms_messager)

    sms_messager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class HomeCommander(Node):

    def __init__(self):
        super().__init__('home_commander')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    home_commander = HomeCommander()

    rclpy.spin(home_commander)

    home_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

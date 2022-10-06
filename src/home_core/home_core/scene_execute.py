# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class SceneExecutive(Node):

    def __init__(self):
        super().__init__('scene_executive')
        self.settings_subscription = self.create_subscription(
            String,
            'settings',
            self.settings_callback,
            10)
        self.settings_subscription  # prevent unused variable warning
        
    def settings_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    scene_executive = SceneExecutive()

    rclpy.spin(scene_executive)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scene_executive.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

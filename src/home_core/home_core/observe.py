# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
import json

class HomeObserver(Node):

    def __init__(self):
        super().__init__('home_observer')
        # measurements
        self.wx_temperature_c = []
        self.wx_wind_speed_kph = []
        self.wx_wind_dir_from_degrees = []
        self.wx_humidity_percent = []
        self.dev_num_discovered_nodes = []

        # subscribe to curent conditions
        self.wx_cond_sub = self.create_subscription(
            String,
            'wx_conditions',
            self.wx_cond_callback,
            10)
        self.wx_cond_sub  # prevent unused variable warning

    def wx_cond_callback(self, msg):
        m = json.loads(msg.data)
        cond = m['payload']
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    home_observer = HomeObserver()

    rclpy.spin(home_observer)

    home_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

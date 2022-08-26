# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class WeatherTracker(Node):

    def __init__(self):
        super().__init__('weather_tracker')
        self.publisher_ = self.create_publisher(String, 'forecast', 10)
        timer_period = 30.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Weather: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    weather_tracker = WeatherTracker()

    rclpy.spin(weather_tracker)

    weather_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

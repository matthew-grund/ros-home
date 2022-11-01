# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json
import requests

class YamahaDevice(Node):

    def __init__(self):
        super().__init__('yamaha_device')
        self.publisher_status = self.create_publisher(String, 'media_status', 10)
        self.poll_timer_period = 0.37  # seconds
        self.poll_timer = self.create_timer(self.poll_timer_period, self.poll_timer_callback)
        self.i = 0
        self.do_need_device_msg = True
        self.yamaha_ip_addresses = []
        
    def poll_timer_callback(self):
        if self.do_need_device_msg:
            return

        m = {}
        m['index'] = self.i
        m['interval'] = self.poll_timer_period
        p={}
        p['yamaha_ip'] = ip
        p['type'] = "Yamaha"
        m['payload'] = p
        mstr = json.dumps(m)
        msg = String()
        msg.data = mstr
        self.publisher_status.publish(msg)
        self.get_logger().info('Lutron publishing status for %d lights' % len(self.device_status))
        self.get_logger().info(f"Lutron device status: %s" % mstr)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    yamaha_device = YamahaDevice()

    rclpy.spin(yamaha_device)

    yamaha_device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

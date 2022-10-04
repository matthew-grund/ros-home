# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import bluetooth    
import json

class OccupancyTracker(Node):
    #  The occupancy_tracker node uses bluetooth discovery methods, and other 
    #  methods. The node publishes data on the "occupants" topic, which is
    #  monitored by the event_detect node.
    #  The stretch goal is per-room occupancy detection, however the minimum requirement
    #  is home/away detection for all family members.
    def __init__(self):
        super().__init__('occupancy_tracker')
        self.publisher_ = self.create_publisher(String, 'occupants', 10)
        self.bt_timer_period = 3.0  # seconds
        self.timer_bt = self.create_timer(self.bt_timer_period, self.bt_timer_callback)
        self.n_bt = 0

    def bt_timer_callback(self):
        nearby_devices =  bluetooth.discover_devices()
        msg = String()
        msg.data = 'Occupants: %d' % self.n_bt
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.n_bt += 1


def main(args=None):
    rclpy.init(args=args)

    occupancy_tracker = OccupancyTracker()

    rclpy.spin(occupancy_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupancy_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

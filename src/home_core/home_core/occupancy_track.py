# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import json
import math

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

        self.config_subscription = self.create_subscription(
            String,
            '/home/configuration',
            self.config_listener_callback,
            10)
        self.config_subscription
        self.need_config_location = True
        self.need_config_occupants = True

        self.owntracks_subscription = self.create_subscription(
            String,
            '/people/tracking/owntracks',
            self.owntracks_listener_callback,
            10)
        self.config_subscription
        self.need_config_location = True
        self.need_config_occupants = True
        self.occupants_are_home = {}


    def config_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "LOCATION":
            self.home_latitude = float(msg['payload']['Coordinates']['latitude']) 
            self.home_longitude = float(msg['payload']['Coordinates']['longitude']) 
            self.need_config_location = False
            self.get_logger().info(f"Got config: Location is {self.city}, {self.state}")
        # todo: add OCCUPANTS config

    def owntracks_listener_callback(self,msg):
        new_occ = False
        m = json.loads(msg.data)
        if m['type'] == 'EVENT' and  m['action'] == 'INFILTRATE' and m['region'].upper() == "HOME":
            self.occupants_are_home[m['tracked_id']] = True
            new_occ = True
        if m['type'] == 'EVENT' and m['action'] == 'EXFILTRATE' and m['region'].upper() == "HOME":
            self.occupants_are_home[m['tracked_id']] = False
            new_occ = True
        if new_occ:
            self.publish_occupancy()
            _    
    def publish_occupancy(self):
        n_home = 0
        n_occ = 0
        occ_list = []
        for o in self.occupants_are_home:
            n_occ += 1
            if self.occupants_are_home[o]:    
                n_home += 1
        
                
    def bt_timer_callback(self):
        # nearby_devices =  bluetooth.discover_devices()
        msg = String()
        msg.data = 'Occupants: %d' % self.n_bt
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.n_bt += 1


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

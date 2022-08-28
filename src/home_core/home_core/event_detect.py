# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#

from cmath import nan
from distutils.log import set_verbosity
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import datetime

class EventDetector(Node):

    def __init__(self):
        super().__init__('home_event_detect')
        self.new_devices=[]
        self.known_devices=[]
        self.subscription_devices = self.create_subscription(
            String,
            'devices',
            self.devices_callback,
            10)
        self.subscription_devices  # prevent unused variable warning

        self.subscription_occupancy = self.create_subscription(
            String,
            'occupants',
            self.occupancy_callback,
            10)
        self.subscription_occupancy  # prevent unused variable warning

        self.subscription_sun = self.create_subscription(
            String,
            'sun',
            self.sun_callback,
            10)
        self.subscription_sun  # prevent unused variable warning

        self.subscription_wx = self.create_subscription(
            String,
            'weather',
            self.wx_callback,
            10)
        self.subscription_wx  # prevent unused variable warning

        self.publisher_events = self.create_publisher(String, 'events', 10)
        self.period_name = ""
        timer_period = 10.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.dev_index = 0
        self.event_index = 0
        self.publisher_devices = self.create_publisher(String, 'known_devices', 10)

    def timer_callback(self):
        self.i += 1
    
    def publish_event(self, typename, severity, desc, detail):
        msg = String()
        event = {}
        event['event_type'] = typename
        event['severity'] = severity
        event['timestamp'] = datetime.datetime.utcnow().isoformat()
        event['description'] = desc
        event['detail'] = detail
        m = {}
        m['index'] = self.event_index
        m['interval'] = nan
        m['payload'] = event
        msg.data = json.dumps(m)
        self.publisher_events.publish(msg)
        if severity=='ERROR':
            self.get_logger().error(desc)
        elif severity=='WARNING':
            self.get_logger().warn(desc)
        else:
            self.get_logger().info(desc)
        self.event_index += 1    

    def publish_known_devices(self):
        msg = String()
        msgdict = {}
        msgdict['index'] = self.i
        msgdict['interval'] = self.device_interval
        msgdict['payload'] = self.known_devices
        msg.data = json.dumps(msgdict)
        self.publisher_devices.publish(msg)
        self.get_logger().info('Devices: Publishing known devices list: %d devices' % len(self.known_devices))

    def devices_callback(self, msg):
        m = json.loads(msg.data)
        self.device_interval = m['interval']
        self.new_devices = m['payload']
        if m['index'] == 0:
            self.publish_event('DEV','ERROR','Device discovery node restarted',self.new_devices)
        self.detect_device_event()

    def occupancy_callback(self, msg):
        self.get_logger().info('Occupants: "%s"' % msg.data)

    def sun_callback(self, msg):
        m = json.loads(msg.data)
        max_sec = m['interval']
        sun = m['payload']
        if m['index'] == 0:
            self.publish_event('SUN','ERROR','Sun tracker node restarted',sun)
        rem_hours = sun['secs_remaining'] / 3600.0
        event = sun['next_event']    
        self.get_logger().info('Sun: %.1f hours until %s' % (rem_hours,event))
        if sun['secs_remaining'] <= max_sec:
            self.publish_event('SUN','INFO','%s'% event,sun)

    def wx_callback(self, msg):
        m = json.loads(msg.data)
        max_sec = m['interval']
        wx = m['payload']
        if m['index'] == 0:
            self.publish_event('WEATHER','ERROR','Weather tracker node restarted',wx)
            self.period_name = ""
        if self.period_name != wx[0]['name']:
            self.publish_event('WEATHER','INFO','NEW FORECAST: %s - %s'% (wx[0]['name'],wx[0]['detailedForecast']),wx)
            self.period_name = wx[0]['name']
        else:
            self.get_logger().info('Weather: %s - %s' % (wx[0]['name'],wx[0]['detailedForecast']))

    def detect_device_event(self):
        known_len = len(self.known_devices)
        new_len = len(self.new_devices)
        if known_len == 0:
            self.publish_event('DEV','INFO','Discovered %d NEW devices' % new_len,self.new_devices)
        for d in self.new_devices:
            dev_is_new = True
            for kd in self.known_devices:
                if d['ip'] == kd['ip']:
                    kd['times_seen'] = kd['times_seen'] + 1
                    kd['last_seen'] = datetime.datetime.utcnow().isoformat()
                    dev_is_new = False
                    # TODO: merge the device data, preserving extra keys, known over 'Unknown' values
            if dev_is_new:
                d['times_seen'] = 1
                d['times_missed'] = 0
                d['last_seen'] = datetime.datetime.utcnow().isoformat()
                self.known_devices.append(d)
                self.publish_event('DEV','INFO','Discovered NEW device: %s@%s' % (d['name'],d['ip']),d)
        # check if a known device is missing from the lastest scan        
        for kd in self.known_devices:
            dev_is_lost = True
            for d in self.new_devices:
                if d['ip'] == kd['ip']:
                    dev_is_lost = False
            if dev_is_lost:
                kd['times_missed'] = kd['times_missed'] + 1
                self.publish_event('DEV','WARNING','Missed known device: %s@%s last seen %s' % (kd['name'],kd['ip'],kd['last_seen']),kd)
        # share the master list        
        self.publish_known_devices()

def main(args=None):
    rclpy.init(args=args)

    home_event_detect = EventDetector()

    rclpy.spin(home_event_detect)

    home_event_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

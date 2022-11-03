# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
from cmath import nan
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
import json
import bluetooth
from bluetooth.ble import DiscoveryService
from datetime import date, time, datetime, timedelta
from dateutil import parser
import math
import pytz
class BluetoothDeviceDiscoverer(Node):

    def __init__(self):
        super().__init__('bluetooth_discover')
        self.publisher_devices = self.create_publisher(String, '/devices/bluetooth_devices', 10)
        self.scan_period = 33.0  # seconds
        self.host_timer = self.create_timer(self.scan_period, self.host_scan_callback)
        self.scan_index = 0
        self.msg_index  = 0
        self.host_disco_svc = DiscoveryService()
        # subscribe to settings
        self.settings_sub = self.create_subscription(
            String,
            '/home/configuration',
            self.settings_callback,
            10)
        self.settings_sub  # prevent unused variable warning
        self.devices = {}  # dictionary of known devices

    def settings_callback(self, msg):
        m = json.loads(msg.data)
        wx = m['payload']
        temp = wx
        self.get_logger().info('Settings: "%s"' % msg.data)
        

    def host_scan_callback(self):
        device_list_changed = False
        self.scan_index += 1
        newdevs = self.host_disco_svc.discover(13) 
        now = datetime.now()
        midnight = datetime.combine(datetime.today(), time.min)
        daysec = (now - midnight).total_seconds()  # seconds since midnight today
        for dev in newdevs:
            if dev in self.devices:
                if len(self.devices[dev]['name']) < len(newdevs[dev]):   # keep the longest name scanned (some scans have '' for a named device)
                    self.devices[dev]['name'] = newdevs[dev] 
                    self.get_logger().info('Bluetooth: got name for: %s %s' % (dev, newdevs[dev]))   
                self.devices[dev]['last_seen'] = daysec
            else:
                # a new device found  
                newdevinfo = {}      
                newdevinfo['last_seen'] = daysec
                if len(newdevs[dev]):
                    name = newdevs[dev]
                else:    
                    name = bluetooth.lookup_name(dev,7)
                    if name == None:
                        name = ""
                    self.get_logger().warn('Bluetooth: device: %s name not discovered - using "%s"' % (dev,name))   
                newdevinfo['name'] = name    # whatever the initial scan gave (usually "")
                self.devices[dev] = newdevinfo
                self.get_logger().info('Bluetooth: new device: %s %s' % (dev,json.dumps(newdevinfo)))   
                device_list_changed = True
                
        # forget old devices
        device_list_changed |= self.expire_devices()

        # try to get names of yet unnamed devices
        device_list_changed |= self.query_device_names()
        
        if device_list_changed:
            self.publish_device_list()
    
    def expire_devices(self):
        now = datetime.now()
        stale = False
        midnight = datetime.combine(datetime.today(), time.min)
        daysec = (midnight - now).total_seconds()  # seconds since midnight today
        for dev in self.devices:
            if daysec - self.devices[dev]['last_seen'] > 60.0:
                # remove stale device         
                stale = True
                del self.devices[dev]
                self.get_logger().warn('Bluetooth: lost device: %s %s' % (dev,json.dumps(self.devices[dev])))   
        return stale

    def query_device_names(self):
        now = datetime.now()
        got_names = False
        midnight = datetime.combine(datetime.today(), time.min)
        daysec = (midnight - now).total_seconds()  # seconds since midnight today
        for dev in self.devices:
            if self.devices[dev]['name'] == "":
                name = bluetooth.lookup_name(dev, 7)
                if name != None:
                    got_names = True
                    self.devices[dev]['name'] = name
                    self.get_logger().info('Bluetooth: device: %s has name %s' % (dev,name))
        return got_names

    def publish_device_list(self):
        msg = String()
        msgdict = {}
        msgdict['index'] = self.msg_index
        msgdict['interval'] = nan
        msgdict['payload'] = self.devices
        msg.data = json.dumps(msgdict)
        self.publisher_devices.publish(msg)
        self.get_logger().info('Bluetooth: Publishing known devices list: %d devices: %s' % (len(self.devices),json.dumps(self.devices)))
        self.msg_index += 1
        
def main(args=None):
    rclpy.init(args=args)

    bt_discoverer = BluetoothDeviceDiscoverer()

    rclpy.spin(bt_discoverer)

    bt_discoverer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

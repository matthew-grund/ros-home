#  Copyright 2022 Matthew Grund
#
#  Licensed under the BSD 2 Clause license;
#  you may not use this file except in compliance with the License.
#
from telnetlib import STATUS
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import json
import requests

class YamahaDevice(Node):

    def __init__(self):
        super().__init__('yamaha_device')
        self.publisher_status = self.create_publisher(String, 'media_status', 10)
        self.poll_timer_period = 1  # seconds
        self.poll_timer = self.create_timer(self.poll_timer_period, self.poll_timer_callback)
        self.i = 0
        self.do_need_device_msg = True
        self.yamaha_devices = {}  #  dict with IP keys

        self.config_sub = self.create_subscription(
            String, 'settings',
            self.config_callback,10)

        self.command_sub = self.create_subscription(
            String,'media_commands',
            self.command_callback,10)

        self.command_sub = self.create_subscription(
            String,'events',
            self.event_callback,10)

        self.devices_sub = self.create_subscription(
            String,'known_net_devices',
            self.devices_callback,10)

    
    def poll_timer_callback(self):
        if self.do_need_device_msg:
            return
        if len(self.yamaha_devices) == 0:
            return
        for ip in self.yamaha_devices:
            base_url = f"http://{ip}/YamahaExtendedControl/"
            # status
            get_status_url =  base_url + "v1/main/getStatus"
            try:    
                ret = requests.get(get_status_url)
            except:
                return {} # FIXME - a better error value?
            if ret.status_code == 200:      # success! 
                status = json.loads(ret.text)
                self.yamaha_devices[ip]['status'] = status
                self.get_logger().info(f"Yamaha: got status for {self.yamaha_devices[ip]['name']}: power:{status['power']} volume:{status['actual_volume']['value']} dB")  
            # play info
            get_play_url = base_url + "v1/netusb/getPlayInfo"
            try:    
                ret = requests.get(get_play_url)
            except:
                return {} # FIXME - a better error value?
            if ret.status_code == 200:      # success!
                play = json.loads(ret.text)
                self.yamaha_devices[ip]['play'] = play
                self.get_logger().info(f"Yamaha: got play info for {self.yamaha_devices[ip]['name']}: {play['input']}::{play['track']}:::{play['playback']} ")     
            # publish an update    
            m = {}
            m['index'] = self.i
            m['interval'] = self.poll_timer_period
            p={}
            p['type'] = "Yamaha"
            m['payload'] = self.yamaha_devices[ip]
            mstr = json.dumps(m)
            msg = String()
            msg.data = mstr
            self.publisher_status.publish(msg)
            self.get_logger().info(f"Yamaha publishing update for {self.yamaha_devices[ip]['name']}")
            self.i += 1
        
    def config_callback(self,msg):
        return 
       
    def command_callback(self,msg):
        return 
           
    def event_callback(self,msg):
        return 
    
    def devices_callback(self,msg):
        m = json.loads(msg.data)
        self.do_need_device_msg = False
        devices = m['payload']
        for dev in devices:
            if dev['vendor'] == "Yamaha Corporation":
                if dev['ip'] not in self.yamaha_devices:
                    self.yamaha_devices[dev['ip']] = dev
                    self.get_logger().info(f"Yamaha: new device {dev['name']} found. ip = {dev['ip']}")

          
def main(args=None):
    rclpy.init(args=args)

    yamaha_device = YamahaDevice()

    rclpy.spin(yamaha_device)

    yamaha_device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

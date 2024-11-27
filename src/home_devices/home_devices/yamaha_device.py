#  Copyright 2022 Matthew Grund
#
#  Licensed under the BSD 2 Clause license;
#  you may not use this file except in compliance with the License.
#
# from telnetlib import STATUS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests

class YamahaDevice(Node):
    #
    #  single instance tracks multiple yamaha amplifiers
    #
    def __init__(self):
        super().__init__('yamaha_device')
        self.publisher_status = self.create_publisher(String, '/media/status', 10)
        self.poll_timer_period = 1  # seconds
        self.poll_timer = self.create_timer(self.poll_timer_period, self.poll_timer_callback)
        self.i = 0
        self.do_need_device_msg = True
        self.do_need_config_msg = True
        self.yamaha_devices = {}  #  dict with IP address information

        self.config_sub = self.create_subscription(
            String,
            '/home/configuration',
            self.config_callback,
            10)
        self.config_sub  # prevent unused variable warning

        self.command_sub = self.create_subscription(
            String,'/media/commands',
            self.command_callback,10)

        self.command_sub = self.create_subscription(
            String,'/diagnostics/events',
            self.event_callback,10)

        self.devices_sub = self.create_subscription(
            String,'/devices/known/network',
            self.devices_callback,10)

    
    def poll_timer_callback(self):
        # we don't have any configuration updates yet
        if self.do_need_device_msg and self.do_need_config_msg:
            return
        # we don't know of any yamaha IP addresses
        if len(self.yamaha_devices) == 0:
            return
        # get to here, we've got amps
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
                self.get_logger().info(f"Yamaha: got status for {self.yamaha_devices[ip]['room']}: status:{status['power']} volume:{status['actual_volume']['value']} dB")
                # self.get_logger().info(f"Yamaha: got status for {ip}: {status}")  
            if status['power'] == 'on':     
                # play info
                get_play_url = base_url + "v1/netusb/getPlayInfo"
                try:    
                    ret = requests.get(get_play_url)
                except:
                    return {} # FIXME - a better error value?
                if ret.status_code == 200:      # success!
                    play = json.loads(ret.text)
                    self.yamaha_devices[ip]['play'] = play
                    self.get_logger().info(f"Yamaha: got playback info for {self.yamaha_devices[ip]['room']}: {play['input']}::{play['track']}::{play['playback']} ") 
                    # self.get_logger().info(f"Yamaha: got playback info for {ip}: {play['input']}::{play['track']}:::{play['playback']} ") 
            else:
                self.yamaha_devices[ip]['play'] = {}
                
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
            self.get_logger().info(f"Yamaha publishing update for {self.yamaha_devices[ip]['room']}")
            # self.get_logger().info(f"Yamaha publishing update for {ip}")
            self.i += 1
        

    def config_callback(self,msg):
        # parse a home configuration message
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "YAMAHA":
            self.do_need_config_msg = False
            payload = msg['payload']
            self.get_logger().info(f"Yamaha got config message type YAMAHA {msg}")
            for section in payload:
                if self.is_ip_address(section):
                    # not a new address
                    if section in self.yamaha_devices:
                        self.get_logger().warning(f"Yamaha got config for existing address '{section}' not adding")
                        return
                    # get to here it's a new amp address
                    self.yamaha_devices[section]=payload[section]
                    self.get_logger().info(f"Yamaha got config for address '{section}' = {payload[section]}")
                else:
                    self.get_logger().warning(f"Yamaha found config section '{section}' - ignoring")
        else:
            self.get_logger().warning(f"Yamaha got config for {msg['payload']['type']}' - ignoring")    
        return 
       
    def command_callback(self,msg):
        return 
           
    def event_callback(self,msg):
        return 
    
    def devices_callback(self,msg):
        #
        #   Parses a network devices message, typically written to /devices/known/network by net_discover.py
        #
        m = json.loads(msg.data)
        self.do_need_device_msg = False
        devices = m['payload']
        for dev in devices:
            if dev['vendor'] == "Yamaha Corporation":
                if dev['ip'] not in self.yamaha_devices:
                    self.yamaha_devices[dev['ip']] = dev
                    self.get_logger().info(f"Yamaha: new device {dev['name']} found. ip = {dev['ip']}")


    def is_ip_address(self,addr:String):
        toks = addr.split(".")
        if len(toks) != 4:
            return False
        for tok in toks:
            if tok.isnumeric() is False:
                return False
        return True

          
def main(args=None):
    rclpy.init(args=args)

    yamaha_device = YamahaDevice()

    rclpy.spin(yamaha_device)

    yamaha_device.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

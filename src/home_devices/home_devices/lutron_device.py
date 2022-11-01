# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import asyncio
from pylutron_caseta.smartbridge import Smartbridge


class LutronDevice(Node):
    def __init__(self):
        super().__init__('lutron_device')
        self.publisher_status = self.create_publisher(String, 'lighting_status', 10)
        self.poll_timer_period = 6  # seconds
        self.poll_timer = self.create_timer(self.poll_timer_period, self.poll_timer_callback)
        self.i = 0
        self.num_commands = 0 
        self.lutron_keyfile = ""
        self.lutron_certfile = ""
        self.lutron_bridgecertfile = ""
        self.lutron_ipaddress = ""
        self.do_need_config_msg = True
        self.raw_device_status = []
        self.rooms = []
        self.lights = []
        self.device_status = {}
        
        # subscribe to config
        self.config_sub = self.create_subscription(
            String,
            'settings',
            self.config_callback,
            10)
        self.config_sub  # prevent unused variable warning
                
        # subscribe to commands
        self.command_sub = self.create_subscription(
            String,
            'lighting_commands',
            self.command_callback,
            10)
        self.command_sub  # prevent unused variable warning
        # make  an asyncio event loop
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
    #  LUTRON.INI: {"index": 8, 
    #               "interval": 0.5, 
    #               "payload": {"filepath": "/data/home_ws/config/lutron.ini", 
    #                           "type": "LUTRON", 
    #                           "Bridge": {"address": "10.0.0.180"}, 
    #                           "Key": {"filepath": "/data/home_ws/aux/10.0.0.180.key"}, 
    #                           "Certificate": {"filepath": "/data/home_ws/aux/10.0.0.180.cert"}, 
    #                           "Bridge Certificate": {"filepath": "/data/home_ws/aux/10.0.0.180-bridge.crt"}}}
    def config_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "LUTRON":
            self.lutron_ipaddress = msg['payload']['Bridge']['address']
            self.lutron_certfile = msg['payload']['Certificate']['filepath']
            self.lutron_keyfile = msg['payload']['Key']['filepath']
            self.lutron_bridgecertfile = msg['payload']['Bridge Certificate']['filepath']
            self.do_need_config_msg = False
            self.get_logger().info(f"Got config: SmartBridge is {self.lutron_ipaddress}" )


    async def set(self,id,value):
        if self.do_need_config_msg:
            return        
        bridge = Smartbridge.create_tls(self.lutron_ipaddress, self.lutron_keyfile, self.lutron_certfile, self.lutron_bridgecertfile)    
        await bridge.connect()
        ret = bridge.set_value(str(id),int(value))
        await bridge.close()
   

    def command_callback(self,msg):
        self.num_commands += 1     
        if self.do_need_config_msg:
            self.get_logger().error(f'Lutron got lighting command before settings message!')
            return
        self.get_logger().info(f'Lutron got lighting command %s' % msg.data)
             
        
    async def poll(self):
        if self.do_need_config_msg:
            return
        bridge = Smartbridge.create_tls(self.lutron_ipaddress, self.lutron_keyfile, self.lutron_certfile, self.lutron_bridgecertfile)
        try:    
            await bridge.connect()
        except asyncio.TimeoutError:   
            self.raw_device_status = {}
            self.get_logger().warn(f"Lutron: timeout while fetching device status")                    
        try:
            self.raw_device_status = bridge.get_devices()
        except asyncio.TimeoutError:
            self.raw_device_status = {}
            self.get_logger().warn(f"Lutron: timeout while fetching device status")    
        try:     
            await bridge.close()
        except asyncio.TimeoutError:
            self.raw_device_status = {}
            self.get_logger().warn(f"Lutron: timeout while fetching device status")  
            
    def poll_timer_callback(self):
        if self.do_need_config_msg:
            return
        self.loop.run_until_complete(self.poll())
        if len(self.raw_device_status) == 0:
            return
        self.process_raw_status()
        m = {}
        m['index'] = self.i
        m['interval'] = self.poll_timer_period
        p={}
        p['hub_ip'] = self.lutron_ipaddress
        p['type'] = "Lutron"
        p['lights'] = self.device_status
        m['payload'] = p
        mstr = json.dumps(m)
        msg = String()
        msg.data = mstr
        self.publisher_status.publish(msg)
        self.get_logger().info('Lutron publishing status for %d lights' % len(self.device_status))
        self.get_logger().info(f"Lutron device status: %s" % mstr)
        self.i += 1
        
    def process_raw_status(self):
        devs = []
        for device_id in self.raw_device_status:
            if self.raw_device_status[device_id]['type'] != "SmartBridge":
                ds={}
                ds['id'] = device_id
                ds['type'] = self.raw_device_status[device_id]['type']
                ds['state'] = self.raw_device_status[device_id]['current_state']
                rawname = self.raw_device_status[device_id]['name']
                names = rawname.split('_')
                ds['room'] = names[0]
                if names[0] not in self.rooms:
                    self.rooms.append(names[0])
                ds['name'] = names[1]
                if names[1] not in self.lights:
                    self.lights.append(names[1])
                devs.append(ds)
        self.device_status = devs
        
    def isnumber(self,str_to_check):
        try:
            float(str_to_check)
            return True
        except ValueError:
            return False
            
def main(args=None):
    rclpy.init(args=args)

    lutron_device = LutronDevice()

    rclpy.spin(lutron_device)

    lutron_device.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
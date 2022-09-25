# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import asyncio
from pylutron_caseta.smartbridge import Smartbridge

async def run_lutron(address,key,cert,bridge,ros2_node):
    bridge = Smartbridge.create_tls(
        "YOUR_BRIDGE_IP", "caseta.key", "caseta.crt", "caseta-bridge.crt"
    )

class LutronDevice(Node):
    def __init__(self):
        super().__init__('lutron_device')
        self.publisher_ = self.create_publisher(String, 'lutron_device_status', 10)
        timer_period = 2.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.lutron_keyfile = ""
        self.lutron_certfile = ""
        self.lutron_bridgecertfile = ""
        self.lutron_ipaddress = ""
        self.do_need_config_msg = True
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
            'commands',
            self.commannd_callback,
            10)
        self.command_sub  # prevent unused variable warning
        
    def timer_callback(self):
        msg = String()
        msg.data = 'Light Status: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def need_config_msg(self):
        return self.do_need_config_msg
    
    
async def ros_spin_once(minimal_publisher):
    rclpy.spin_once(minimal_publisher, timeout_sec=0)

async def run_ros_node(ros2_node):
    while True:
        await ros_spin_once(ros2_node)
        await asyncio.sleep(0)

async def main(args=None):
    rclpy.init(args=args)
    device = LutronDevice()
    task_ros2_node = asyncio.create_task(run_ros_node(device))
    # just the ros task runs, until we get the config, and can open the lutron
    while device.need_config_msg():
        await asyncio.sleep(0)
    task_lutron_bridge = asyncio.create_task(run_lutron(device))
    await asyncio.gather(task_ros2_node, task_lutron_bridge)
    device.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())

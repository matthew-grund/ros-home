#
#  ROS Home UI Node - a bridge between ROS Home and various UIs
#
# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
from cmath import nan
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import date, time, datetime, timedelta
import json
from dateutil import parser
import math
import sys
import os
import urllib.request
from types import NoneType

class ROSHomeUINode(Node):
    def __init__(self):
        super().__init__('rqt_home')
        self.publisher_cmds = self.create_publisher(String, '/people/commands', 10)
        self.config_subscription = self.create_subscription(String,
            '/home/configuration',
            self.config_listener_callback,10)
        self.lighting_subscription = self.create_subscription(String,
            '/lighting/status',
            self.lighting_status_callback,10)    
        self.lighting_subscription = self.create_subscription(String,
            '/lighting/scenes',
            self.lighting_scenes_callback,10)    
        self.event_subscription = self.create_subscription(String,
            '/diagnostics/events',
            self.event_listener_callback,10)
        self.wx_conditions_subscription = self.create_subscription(String,
            '/environment/weather/conditions',
            self.conditions_listener_callback,10)
        self.known_devices_subscription = self.create_subscription(String,
            '/devices/known/network',
            self.devices_listener_callback,10)
        self.nodes_subscription = self.create_subscription(String,
            '/nodes/list',
            self.nodes_listener_callback,10)
        self.media_status_subscription = self.create_subscription(String,
            '/media/status',
            self.media_listener_callback,10)
                                               
        self.spin_count = 0
        self.lighting_msg_count = 0
        self.event_msg_count = 0
        self.conditions_msg_count = 0
        self.devices_msg_count = 0
        self.nodes_msg_count = 0
        self.scene_msg_count = 0
        self.media_msg_count = 0

        
    def send_command(self):
        if len(self.commands):
            for cmd in self.commands:
                m = {}
                m['index'] = self.i
                m['interval'] = math.nan
                p={}
                p['type'] = "USER"
                p['source'] = "rqt_home"
                p['command'] = cmd
                mstr = json.dumps(m)
                msg = String()
                msg.data = mstr
                self.publisher_cmds.publish(msg)
                self.get_logger().info('rqt_home publishing command :%s' % cmd)
            self.commands = []
            
    def config_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "RQTHOME":
            self.do_need_config_msg = False
            self.get_logger().info(f"Got config: {msg}" )

    def lighting_status_callback(self,msg):
        msg = json.loads(msg.data)
        self.get_logger().info(f"Got status for {len(msg['payload']['lights'])} {msg['payload']['type']} lights." )
        self.latest_lighting_msg = msg
        self.lighting_msg_count +=1 

    def lighting_scenes_callback(self,msg):
        msg = json.loads(msg.data)
        self.get_logger().info(f"Got lighting scene update for {msg['payload']['today']}: {msg['payload']['previous_scene']}" )
        self.latest_scene_msg = msg
        self.scene_msg_count +=1 

    def event_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['event_type'] != 'DEVICE':
            self.get_logger().info(f"Got {msg['payload']['severity']} event from {msg['payload']['event_type']}" )
            self.latest_event_msg = msg
            self.event_msg_count +=1 

    def conditions_listener_callback(self,msg):
        m = json.loads(msg.data)
        observations = m['payload']
        if (type(observations['temperature']['value']) == int) or \
            (type(observations['temperature']['value']) == float):
            self.get_logger().info('Current weather: "%s" %.1f F' % (observations['textDescription'],observations['temperature']['value']/5*9+32)) # degrees F
        else:
            self.get_logger().info(f"Current weather:  {observations['textDescription']} temp is {str(type(observations['temperature']['value']))}")            
        self.latest_conditions_msg = m
        self.conditions_msg_count +=1 

    def devices_listener_callback(self,msg):
        msg = json.loads(msg.data)
        self.get_logger().info(f"{len(msg['payload'])} network hosts found on {msg['payload'][0]['type']}")
        self.latest_devices_msg = msg
        self.devices_msg_count +=1 

    def nodes_listener_callback(self,msg):
        msg = json.loads(msg.data)
        self.get_logger().info(f"ROS Home: {len(msg['payload'])} nodes currently running")
        self.latest_nodes_msg = msg
        self.nodes_msg_count +=1 

    def media_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['index'] % 7 == 0:
            self.get_logger().info(f"Media: got update from {msg['payload']['name']}")
        self.latest_media_msg = msg
        self.media_msg_count +=1 

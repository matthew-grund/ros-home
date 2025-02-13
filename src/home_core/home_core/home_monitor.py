# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import json
import subprocess
import os
import datetime

class HomeObserver(Node):

    def __init__(self):
        super().__init__('home_monitor')
        # measurements
        self.wx_temperature_c = []
        self.wx_wind_speed_kph = []
        self.wx_wind_dir_from_degrees = []
        self.wx_humidity_percent = []
        self.known_nodes = {}
        self.current_nodes = {}
        self.prev_num_nodes = 0     
        self.need_user_name = True
        self.need_ros2_path = True
        self.need_startup_notify = True
        self.publisher_nodes = self.create_publisher(Int32, '/home/nodes/count', 10)
        self.publisher_node_list = self.create_publisher(String,'/home/nodes/list', 10)
        self.node_list_iter = 0
        self.ps_timer_period =  0.5 # seconds
        self.ps_timer = self.create_timer(self.ps_timer_period, self.ps_timer_callback)

        self.publisher_events = self.create_publisher(String, '/home/events', 10)
        self.period_name = ""
        self.i = 0
        self.dev_index = 0
        self.event_index = 0

        self.publisher_devices = self.create_publisher(String, '/devices/known/network', 10)
        self.wx_latest_conditions = ""
        self.wx_recent_forecasts = {}
        self.lights = []
        self.num_lights_on = -1


    def publish_event(self, desc):
        typename = 'MONITOR'
        severity = 'WARNING'
        msg = String()
        event = {}
        event['event_type'] = typename
        event['severity'] = severity
        event['timestamp'] = datetime.datetime.utcnow().isoformat()
        event['description'] = desc
        event['detail'] = self.known_nodes
        m = {}
        m['index'] = self.event_index
        m['interval'] = 0.0
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


    def ps_timer_callback(self):
        if self.need_user_name:
            self.get_user_name()
        elif self.need_ros2_path:
            self.get_ros2_path()
        elif self.need_startup_notify:
            self.need_startup_notify = False
            self.publish_event('MONITOR node started.');
        else:    
            # check all running processes for ROS-HOME processes
            procs = self.get_user_processes() 
            self.current_nodes = {}
            # build a list of current nodes
            for proc in procs:
                pid = proc['pid']
                name = proc['name']
                cmdline = ' '.join(proc['cmd'])
                if 'ros' in cmdline:
                    if 'home_core' in cmdline:
                        if self.ros2_path not in cmdline:
                            self.current_nodes[pid]=proc
                            if pid not in self.known_nodes:
                                self.get_logger().info(f"Found new {name.upper()} node pid:{pid}")
                            self.known_nodes[pid] = proc
            # look for nodes that died
            dead_pids = []
            for pid in self.known_nodes:
                name=self.known_nodes[pid]['name']
                if pid not in self.current_nodes:
                    self.publish_event(f"Node {name.upper()} pid:{pid} is no longer running")
                    dead_pids.append(pid)
                    # attempt to restart it
                    self.restart_node(self.known_nodes[pid])
                        
            # de-list nodes that died        
            for pid in dead_pids:
                self.known_nodes.pop(pid)
            self.publish_node_list()
            self.publish_node_count()
            
            
    def publish_node_count(self):     
        msg = Int32()
        msg.data = len(self.known_nodes)
        self.publisher_nodes.publish(msg)     # a number, for plotting, etc.
        self.get_logger().debug(f"Found {msg.data} home nodes running")

        
    def publish_node_list(self):    
        msg = String()
        m = {}
        m['index'] = self.node_list_iter
        m['interval'] = self.ps_timer_period
        m['payload'] = self.known_nodes
        msg.data = json.dumps(m)
        self.publisher_node_list.publish(msg) # a JSON string    
        # print(self.current_nodes)


    def get_user_name(self):
        cmd = ['/usr/bin/whoami']
        ret = subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        uname=ret.stdout.decode('utf-8').strip()
        # print(f"Got user name: '{uname}'")
        if len(uname) > 0 :
            self.user_name = uname
            self.need_user_name = False
            self.get_logger().info(f"Read user name: '{uname}'")
        else:
            self.get_logger().error(f"get_user_name(): User name read failed")
            
            
    def get_ros2_path(self):
        cmd = ['/usr/bin/which','ros2']
        ret = subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        ros2=ret.stdout.decode('utf-8').strip()
        # print(f"Got user name: '{uname}'")
        if len(ros2) > 0 :
            self.ros2_path = ros2
            self.need_ros2_path = False
            self.get_logger().info(f"Read ros2 path: '{ros2}'")
        else:
            self.get_logger().error(f"get_ros2_path(): which ros2 failed")


    def restart_node(self,old_proc):
            n = old_proc['name']
            p = old_proc['pkg']
            self.start_node(n,p)

            
    def start_node(self,name,pkg):
            cmd = f"{self.ros2_path} run {pkg} {nname}"
            # print(f"Spawning command: {cmd}")
            ret = subprocess.Popen([self.ros2_path,"run",p,n],start_new_session=True)
            self.get_logger().info(f"Started {n.upper()} from package {p.upper()}")

            
    def get_user_processes(self):
        rhp=[]
        if self.need_user_name:
            self.get_logger().error(f"get_user_processes(): Username not set")
            return rhp
        cmd = ['ps','axo','uname,pid,stat,cmd']
        ret = subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        psstr=ret.stdout.decode('utf-8')
        pslines=psstr.splitlines()
        pslines=pslines[1:]
        for line in pslines:
            #print(line)
            p={}
            field=line.split()
            #print(field)
            uname=field[0]
            p['uname'] = uname
            p['pid'] = int(field[1])
            p['status'] = field[2]
            cmd = field[3:]
            name = '<unknown>'
            pkg = '<unknown>'
            if len(cmd) >= 2:
                exe_path = cmd[-1]
                if exe_path == '--ros-args':
                    exe_path = cmd[-2]
                exe_parts = exe_path.split('/')
                name = exe_parts[-1]
                if len(exe_parts) > 1:
                    pkg = exe_parts[-2]
            p['name'] = name
            p['pkg'] = pkg
            p['cmd'] = cmd
            cmdstr=" ".join(cmd)
            if uname == self.user_name:
                if 'ps ' not in cmdstr: 
                    rhp.append(p)
        return rhp


def main(args=None):
    rclpy.init(args=args)

    home_observer = HomeObserver()

    rclpy.spin(home_observer) # blocks

    home_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

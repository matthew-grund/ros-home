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
import psutil
class HomeObserver(Node):

    def __init__(self):
        super().__init__('home_monitor')
        # measurements
        self.wx_temperature_c = []
        self.wx_wind_speed_kph = []
        self.wx_wind_dir_from_degrees = []
        self.wx_humidity_percent = []
        self.current_nodes = {}
        self.known_nodes = {}
        self.prev_num_nodes = 0     
        self.publisher_nodes = self.create_publisher(Int32, '/nodes/num_running', 10)
        self.publisher_node_list = self.create_publisher(String,'/nodes/list', 10)
        self.node_list_iter = 0
        self.ps_timer_period =  1.0 # seconds
        self.ps_timer = self.create_timer(self.ps_timer_period, self.ps_timer_callback)


    def ps_timer_callback(self):
        # check all running processes for ROS  HOME  processes
        self.current_nodes = {}
        pids = psutil.pids()
        for pid in pids:
            if psutil.pid_exists(pid):
                try:
                    p = psutil.Process(pid)
                except:
                    self.get_logger().info(f"Process inspection stumbled on pid %s" % pid)
                    return
                else:          
                    p_dict = p.as_dict()
                    p_args = p_dict['cmdline']
                    # print(f"{p.name()} {len(p_args)}::'{p_args}'")
                    if p_args is not None:
                        if len(p_args) == 3:
                            # print(f"{p.name()} {len(p_args)}::'{p_args}'")
                            if ("ros-args" in p_args[2]):
                                # print(f"{p.name()} {len(p_args)}::'{p_args}'")
                                # self.get_logger().info(f"node %s[%d] => %s" % (p.name(), pid, str(p_cl_py))) 
                                self.current_nodes[str(pid)] = { 'name':p.name(),'pid':pid, 'status':p.status(), 'cpu':p.cpu_percent(),
                                                                'mem':p.memory_percent(),'args': p_args }
        if self.node_list_iter % 10 == 0:
            self.get_logger().info(f"Monitor: {len(self.current_nodes)} ROS HOME nodes running. [{self.node_list_iter}]")
        self.node_list_iter += 1
        msg = Int32()
        msg.data = len(self.current_nodes)
        self.publisher_nodes.publish(msg)     # a number, for plotting, etc.
        msg = String()
        m = {}
        m['index'] = self.node_list_iter
        m['interval'] = self.ps_timer_period
        m['payload'] = self.current_nodes
        msg.data = json.dumps(m)
        self.publisher_node_list.publish(msg) # a JSON string    
        # print(self.current_nodes)


def main(args=None):
    rclpy.init(args=args)

    home_observer = HomeObserver()

    rclpy.spin(home_observer) # blocks

    home_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

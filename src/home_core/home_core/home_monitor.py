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
                
        self.publisher_nodes = self.create_publisher(Int32, '/nodes/num_running', 10)
        self.publisher_node_list = self.create_publisher(String,'/nodes/list',10)
        self.node_list_iter = 0
        self.ps_timer_period = 10  # seconds
        self.ps_timer = self.create_timer(self.ps_timer_period, self.ps_timer_callback)
        self.current_nodes = {}
        self.known_nodes = {}
        
        # subscribe to curent conditions
        self.wx_cond_sub = self.create_subscription(
            String,
            '/environment/weather/conditions',
            self.wx_cond_callback,
            10)
        self.wx_cond_sub  # prevent unused variable warning

    def wx_cond_callback(self, msg):
        m = json.loads(msg.data)
        wx = m['payload']
        temp = wx
        self.get_logger().info('I heard: "%s"' % msg.data)

    def ps_timer_callback(self):
        # check all running processes for ROS  HOME  processes
        self.current_nodes = {}
        pids = psutil.pids()
        for pid in pids:
            if psutil.pid_exists(pid):
                p = psutil.Process(pid)
                p_dict = p.as_dict()
                p_cl_py = p_dict['cmdline']
                if "home_ws/install/home_" in str(p_cl_py):
                    # self.get_logger().info(f"node %s[%d] => %s" % (p.name(), pid, str(p_cl_py))) 
                    self.current_nodes[p.name() + '['+ str(pid) + ']' ] = p_cl_py.append(p_dict['status'])
        self.get_logger().info(f"Monitor: %d ROS HOME nodes running." % len(self.current_nodes))
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
                
def main(args=None):
    rclpy.init(args=args)

    home_observer = HomeObserver()

    rclpy.spin(home_observer)

    home_observer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

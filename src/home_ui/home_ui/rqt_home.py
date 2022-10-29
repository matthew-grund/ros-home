#
#  ROS Home Qt UI
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

import PyQt6.QtWidgets as qtw
import PyQt6.QtCore as qtc
import PyQt6.QtGui as qtg

class ROSHomeUI(Node):

    def __init__(self):
        super().__init__('rqt_home')
        self.publisher_cmds = self.create_publisher(String, 'commands', 10)
        self.config_subscription = self.create_subscription(String,
            'settings',
            self.config_listener_callback,10)
        self.config_subscription       
        self.lighting_subscription = self.create_subscription(String,
            'lighting_status',
            self.lighting_status_callback,10)    
        self.lighting_subscription
        self.spin_count = 0
        self.lighting_msg_count = 0
        
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
        self.get_logger().info(f"Got lighting status: {msg}" )
        self.lighting_msg_count +=1 


#######################################################################
#
#     The class ROSHomeUI is the main class in this app. It owns a ROS
#     node that gets spin_once() on a QT timer.
#
class RQTHomeUI(qtw.QMainWindow):
    
    def __init__(self):
        self.app = qtw.QApplication(sys.argv)
        qtw.QMainWindow.__init__(self)
        
        self.title = "ROS Home"
        self.description = "Interface to ROS Home"
        self.screen = self.app.primaryScreen()
        self.screen_size =  self.screen.size()
        self.setWindowTitle(self.title)
        self.resize(
            int(self.screen_size.width()*0.5),
            int(self.screen_size.height()*0.87)
            )
        
        self.setup_actions()
        self.setup_top_menu()
        
        self.node = ROSHomeUI()
        self.ros_spin_timer = qtc.QTimer()
        self.ros_spin_timer.singleShot(50, self.ros_action)
        self.ros_spin_count = 0
        
    def ros_action(self):
        # do ros stuff, then check for ros results
        rclpy.spin_once(self.node,timeout_sec=0.100) 
        self.node.spin_count += 1 
        if self.node.spin_count > 37:
            print(f"total light messages: %d" % self.node.lighting_msg_count)   
            self.node.spin_count = 0 
        self.ros_spin_timer.singleShot(50,self.ros_action)
        
    def ui_action(self,parent_label, label):
        a = qtg.QAction(label,self)
        a.triggered.connect(lambda : self.ui_act(parent_label,label))
        a.setEnabled(True)
        
        if parent_label in self.action_dict:
            self.action_dict[parent_label][label] = a 
        else:
            self.action_dict[parent_label] = {}
            self.action_dict[parent_label][label] = a
        
    def setup_actions(self): 
        self.action_dict = {}
        # home menu
        self.ui_action("home","overview")
        self.ui_action("home","location")
        self.ui_action("home","restart system")
        # people menu
        self.ui_action("people","view")
        self.ui_action("people","add")       
        self.ui_action("people","track")
        # lighting menu
        self.ui_action("lighting","view")
        self.ui_action("lighting","scenes")
        self.ui_action("lighting","shedule")
        self.ui_action("lighting", "triggers")
        # temperature menu
        self.ui_action("temperature","view")
        self.ui_action("temperature","modes")
        self.ui_action("temperature","schedule")
        self.ui_action("temperature","triggers")
        # security menu
        self.ui_action("security","view")
        self.ui_action("security","arm")
        self.ui_action("security","disarm")
        self.ui_action("security","schedule")
        # media menu
        self.ui_action("media","view")
        self.ui_action("media","stagings")
        self.ui_action("media","schedule")
        self.ui_action("media", "triggers")
        # diagnostics
        self.ui_action("diagnostics","console")
        self.ui_action("diagnostics","measures")
        # nodes
        self.ui_action("nodes","status")
        self.ui_action("nodes","messages")
        self.ui_action("nodes","run...")
        # help
        self.ui_action("help","getting started")
        self.ui_action("help","about")
    
    def ui_act(self,parent_name, action_name):   
        print(parent_name + ":" + action_name)
                       
    def setup_top_menu(self):
        self.top_menu = self.menuBar()   
        self.style_menu(self.top_menu)
        # just loop through the action dict, populating the menu
        for menu in self.action_dict:
            m = self.top_menu.addMenu(menu)
            self.style_menu(m)
            for action in self.action_dict[menu]:
                m.addAction(self.action_dict[menu][action])
                
    def style_menu(self, menu):
        menu.setStyleSheet(
            '''
            QMenuBar { background-color: #333337 ; color:#ffffff }
            QMenu { background-color: #434347 ; color:#ffffff }
            '''
            )
        font = menu.font()
        font.setPointSize(12)
        menu.setFont(font)
        
def main(args=None):
    rclpy.init(args=args)
    home_ui = RQTHomeUI()
    home_ui.show()
    ret=home_ui.app.exec()
    rclpy.shutdown()
    home_ui.node.destroy_node()
    sys.exit(ret)

if __name__ == '__main__':
    main()        

    
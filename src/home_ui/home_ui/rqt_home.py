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
        self.lighting_subscription = self.create_subscription(String,
            'lighting_status',
            self.lighting_status_callback,10)    
        self.event_subscription = self.create_subscription(String,
            'events',
            self.event_listener_callback,10)
        self.wx_conditions_subscription = self.create_subscription(String,
            'wx_conditions',
            self.conditions_listener_callback,10)
        self.known_devices_subscription = self.create_subscription(String,
            'known_net_devices',
            self.devices_listener_callback,10)
                                               
        self.spin_count = 0
        self.lighting_msg_count = 0
        self.event_msg_count = 0
        self.conditions_msg_count = 0
        self.devices_msg_count = 0
        
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
        self.version_str = "0.7.0" 
        self.copyright_str = "(c) copyright 2022, Matthew Grund"
        self.screen = self.app.primaryScreen()
        self.screen_size =  self.screen.size()
        self.setWindowTitle(self.title)
        self.resize(
            int(self.screen_size.width()*0.75),
            int(self.screen_size.height()*0.5)
            )
        self.last_wx_temp_deg_f = 99 
        # todo: center the app on the screen
        style_sheet ='''
            QWidget { 
                background-color: #23262A ; 
                color: white ;
            }
            #menu_clock {
                background-color: black ; 
                color: white ;
                padding: 3px 3px ;                
            }
            #big_clock {
                color: white ;
                padding: 8px 8px ;  
            }
            QStatusBar { 
                background-color: black ; 
                color: white ;
            }
            QMenuBar {
                background-color: black ; 
                color: white ;
            }
        '''
        self.app.setStyleSheet(style_sheet)
        self.frame_style = qtw.QFrame.Shape.Panel  # Panel for debugging, NoFrame for  aclean look
       
        self.big_clock  = qtw.QLabel()
        self.big_clock.setAccessibleName("big_clock")
        self.big_clock.setObjectName("big_clock")      
        font = self.big_clock.font()
        font.setPointSize(48)
        self.big_clock.setFont(font)
        self.big_clock.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        self.big_clock.setFrameStyle(self.frame_style)
        self.big_date  = qtw.QLabel()
        self.big_date.setAccessibleName("big_date")
        self.big_date.setObjectName("big_date")      
        font = self.big_date.font()
        font.setPointSize(20)
        self.big_date.setFont(font)
        self.big_date.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        self.big_date.setFrameStyle(self.frame_style)       
               
        self.setup_actions()
        self.setup_top_menu()
        self.setup_center_widget()
        self.setup_keyboard_shortcuts()
        
        # splash the version, to start
        self.statusBar().showMessage(self.title + "      ver:" + self.version_str + "       " + self.copyright_str)
        
        self.ros_node = ROSHomeUI()
        self.ros_spin_timer = qtc.QTimer()
        self.ros_spin_timer.singleShot(50, self.ros_timer_callback)
        self.ros_spin_count = 0
        self.clock_tick_timer = qtc.QTimer()
        self.clock_tick_timer.singleShot(200,self.ui_clock_timer_callback)
        
    def ros_timer_callback(self):
        # do ros stuff, then check for ros results
        self.lighting_msg_count = self.ros_node.lighting_msg_count
        self.event_msg_count = self.ros_node.event_msg_count
        self.conditions_msg_count = self.ros_node.conditions_msg_count        
        self.devices_msg_count = self.ros_node.devices_msg_count
        rclpy.spin_once(self.ros_node,timeout_sec=0.100) 
        self.ros_node.spin_count += 1 
        self.check_for_ros_msgs()
        self.ros_spin_timer.singleShot(50,self.ros_timer_callback)
    
    def ui_clock_timer_callback(self):
        now = datetime.now()
        self.clock_str = now.strftime("%H:%M:%S")
        self.date_str = now.strftime("%A %B %-d")
        self.big_clock.setText(self.clock_str)
        self.big_date.setText(self.date_str)
        self.menu_clock.setText(self.clock_str)
        self.clock_tick_timer.singleShot(999,self.ui_clock_timer_callback)
    
    def check_for_ros_msgs(self):
        if self.lighting_msg_count < self.ros_node.lighting_msg_count:
            self.lighting_msg_count = self.ros_node.lighting_msg_count
            self.ui_parse_lighting_msg(self.ros_node.latest_lighting_msg)
        if self.event_msg_count < self.ros_node.event_msg_count:
            self.event_msg_count = self.ros_node.event_msg_count
            self.ui_parse_event_msg(self.ros_node.latest_event_msg)
        if self.conditions_msg_count < self.ros_node.conditions_msg_count:
            self.conditions_msg_count = self.ros_node.conditions_msg_count
            self.ui_parse_conditions_msg(self.ros_node.latest_conditions_msg)    
        if self.devices_msg_count < self.ros_node.devices_msg_count:
            self.devices_msg_count = self.ros_node.devices_msg_count
            self.ui_parse_devices_msg(self.ros_node.latest_devices_msg)    
        
    def ui_parse_lighting_msg(self,msg):
        self.statusBar().showMessage(f"Got status for {len(msg['payload']['lights'])} {msg['payload']['type']} lights.")
        t = datetime.now()
        ts = t.strftime("[%H:%M:%S.%f")[:-3]+']'
        lights = msg['payload']['lights']
        self.num_lights_on = 0
        self.num_lights =  0
        self.lights_total_percent = 0
        for light in lights:
            self.num_lights +=1
            if light['state'] > 0:
               self.lights_total_percent += light['state']
               self.num_lights_on += 1
        if self.num_lights > 0:
            self.lights_total_percent /= self.num_lights 
        self.lighting_summary_label.setText(
            f"{ts}   {msg['payload']['type']}: {self.num_lights_on} of {self.num_lights} lights are on ({int(self.lights_total_percent)}% light energy)") 
        
    def ui_parse_event_msg(self,msg):
        if msg['payload']['event_type'] != 'DEVICE':
            self.statusBar().showMessage(f"Got {msg['payload']['severity']} event from {msg['payload']['event_type']}")
           
    def ui_parse_conditions_msg(self,msg):
        observations = msg['payload']
        if (type(observations['temperature']['value']) == int) or \
            (type(observations['temperature']['value']) == float):
            self.summary_wx_temp_label.setText(f"{int(observations['temperature']['value']/5*9+32)}\u00b0F")
            self.last_wx_temp_deg_f = observations['temperature']['value']/5*9+32
        else:
            self.summary_wx_temp_label.setText(f"{int(self.last_wx_temp_deg_f)}\u00b0F*")                
        self.summary_wx_desc_label.setText(f"{observations['textDescription']}")

    def ui_parse_devices_msg(self,msg):
        t = datetime.now()
        ts = t.strftime("[%H:%M:%S.%f")[:-3]+']'
        num_known = 0
        dev_list = msg['payload']
        for dev in dev_list:
            if dev['is_known']:
                num_known += 1
        self.devices_summary_label.setText(f"{ts}   {len(msg['payload'])} network hosts found on {msg['payload'][0]['type']}: {num_known} known")

    def ui_parse_nodes_msg(self,msg):
        t = datetime.now()
        ts = t.strftime("[%H:%M:%S.%f")[:-3]+']'
        self.nodes_summary_label.setText(f"{ts}   ROS Home: ")
    
    def ui_action(self,parent_label, label):
        a = qtg.QAction(label,self)
        a.triggered.connect(lambda : self.menu_callback(parent_label,label))
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
        self.ui_action("lighting","schedule")
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
        # nodes
        self.ui_action("nodes","status")
        self.ui_action("nodes","messages")
        self.ui_action("nodes","run...")        
        # devices
        self.ui_action("devices","status")
        self.ui_action("devices","messages")
        self.ui_action("devices","all known")
        self.ui_action("devices","identify")     
        # diagnostics
        self.ui_action("diagnostics","measurements")        
        self.ui_action("diagnostics","ros console")
        # help
        self.ui_action("help","getting started")
        self.ui_action("help","about")
    
    def menu_callback(self,parent_name, action_name):   
        print(parent_name + ":" + action_name)
        self.statusBar().showMessage(parent_name + ":" + action_name)
        self.stack.setCurrentIndex(self.frame_dict[parent_name][action_name]['index'])
                       
    def setup_top_menu(self):
        self.top_menu = self.menuBar()  
        self.menu_clock = qtw.QLabel()
        self.menu_clock.setMaximumWidth(72)
        self.menu_clock.setMinimumWidth(72)
        self.menu_clock.setObjectName("menu_clock")
        self.menu_clock.setAccessibleName("menu_clock")
        self.top_menu.setCornerWidget(self.menu_clock)
        # just loop through the action dict, populating the menu
        for menu in self.action_dict:
            m = self.top_menu.addMenu(menu)
            for action in self.action_dict[menu]:
                m.addAction(self.action_dict[menu][action])
            font = m.font()
            font.setPointSize(12)
            m.setFont(font)
        
    def setup_center_widget(self):    
        self.central = qtw.QFrame()
        self.stack = qtw.QStackedLayout()
        self.central.setLayout(self.stack)
        self.frame_dict = {}
        self.num_frames = 0
        # make a pane for each menu item        
        for menu_name in self.action_dict:
            for item_name in self.action_dict[menu_name]:
                self.add_stacked_panel(menu_name, item_name)    
        self.setCentralWidget(self.central)

    def add_stacked_panel(self, menu_name, item_name):
        # frame 
        frame = qtw.QFrame(self.central)
        frame_name_str = menu_name + "_" + item_name + "_frame"
        frame.setAccessibleName(frame_name_str)
        frame.setObjectName(frame_name_str)
        frame.setFrameStyle(self.frame_style)
        # layout
        vert_layout = qtw.QVBoxLayout(frame)
        vert_layout_name_str = menu_name + "_" + item_name + "_layout"
        vert_layout.setObjectName(vert_layout_name_str)
        vert_layout.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        vert_layout.setSpacing(8)
        if self.num_frames == 0:
            vert_layout.addWidget(self.big_clock)
            vert_layout.addWidget(self.big_date)
        else:    
            title_label = qtw.QLabel(frame)
            font = title_label.font()
            font.setPointSize(18)
            title_label.setFont(font)
            title_label.setText(menu_name + " : " + item_name )
            title_label.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
            title_label.setFrameStyle(self.frame_style)
            vert_layout.addWidget(title_label)
        main_horiz_layout = qtw.QHBoxLayout()
        lh_panel = qtw.QFrame()
        #lh_panel.setMinimumSize(64,128)
        lh_panel.setFrameStyle(self.frame_style)
        lh_panel.setObjectName(menu_name + "_" + item_name + "_lh_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("lh_panel")        
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        lh_panel.setLayout(layout)   
        center_panel = qtw.QFrame()
        # center_panel.setMinimumSize(64,128)
        center_panel.setFrameStyle(self.frame_style)
        center_panel.setObjectName(menu_name + "_" + item_name + "_center_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("center_panel")        
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        center_panel.setLayout(layout)   
        rh_panel = qtw.QFrame()
        #rh_panel.setMinimumSize(64,128)
        rh_panel.setFrameStyle(self.frame_style)
        rh_panel.setObjectName(menu_name + "_" + item_name + "_rh_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("rh_panel")        
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        rh_panel.setLayout(layout)   
        main_horiz_layout.addWidget(lh_panel)
        main_horiz_layout.addWidget(center_panel)
        main_horiz_layout.addWidget(rh_panel)
        main_horiz_layout.setSpacing(128)
        footer_panel = qtw.QFrame()
        # footer_panel.setMinimumHeight(64)
        # footer_panel.setMaximumHeight(64)
        footer_panel.setFrameStyle(self.frame_style)
        footer_panel.setObjectName(menu_name + "_" + item_name + "_footer_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("footer_panel")
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        footer_panel.setLayout(layout)       
        # bring it on home
        vert_layout.addLayout(main_horiz_layout)
        vert_layout.addWidget(footer_panel)
        frame.setLayout(vert_layout)
        self.stack.addWidget(frame)
        
        if menu_name not in self.frame_dict:
            self.frame_dict[menu_name] = {}
        self.frame_dict[menu_name][item_name] = {}
        self.frame_dict[menu_name][item_name]['frame'] = frame
        self.frame_dict[menu_name][item_name]['vert_layout'] = vert_layout
        if self.num_frames == 0:
            self.frame_dict[menu_name][item_name]['title_label'] = self.big_clock
        else:    
            self.frame_dict[menu_name][item_name]['title_label'] = title_label
        self.frame_dict[menu_name][item_name]['main_horiz_layout'] = main_horiz_layout
        self.frame_dict[menu_name][item_name]['lh_panel'] = lh_panel
        self.frame_dict[menu_name][item_name]['center_panel'] = center_panel
        self.frame_dict[menu_name][item_name]['rh_panel'] = rh_panel
        self.frame_dict[menu_name][item_name]['footer'] = footer_panel
        self.frame_dict[menu_name][item_name]['index'] = self.num_frames
        self.num_frames += 1
        custom_method_name = "setup_frame_" + menu_name.replace(' ','_') + "_" + item_name.replace(' ','_')
        if hasattr(self, custom_method_name):
            customize = getattr(self,custom_method_name)
            if callable(customize):
                # print(f"found customization method: {custom_method_name}")
                customize(self.frame_dict[menu_name][item_name])
        
    def setup_frame_home_overview(self,f):
        frame = f['footer']
        layout = frame.layout()
        num_widgets = layout.count()
        # remove the last/only widget, then add summary messages
        if (num_widgets):
            layout.removeWidget(layout.itemAt(num_widgets-1).widget())
        self.lighting_summary_label = qtw.QLabel()
        self.lighting_summary_label.setFrameStyle(self.frame_style)
        layout.addWidget(self.lighting_summary_label)
        self.nodes_summary_label = qtw.QLabel()
        self.nodes_summary_label.setFrameStyle(self.frame_style)
        layout.addWidget(self.nodes_summary_label)
        self.devices_summary_label = qtw.QLabel()
        self.devices_summary_label.setFrameStyle(self.frame_style)
        layout.addWidget(self.devices_summary_label)
        frame = f['lh_panel']
        layout = frame.layout()
        num_widgets = layout.count()    
         # use the last/only widget for lighting summary
        if (num_widgets):
            layout.removeWidget(layout.itemAt(num_widgets-1).widget())
        self.summary_wx_temp_label = qtw.QLabel()
        self.summary_wx_temp_label.setAccessibleName("wx_temp")
        self.summary_wx_temp_label.setObjectName("wx_temp")      
        font = self.summary_wx_temp_label.font()
        font.setPointSize(64)
        self.summary_wx_temp_label.setFont(font)
        self.summary_wx_temp_label.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        self.summary_wx_temp_label.setFrameStyle(self.frame_style)  
        layout.addWidget(self.summary_wx_temp_label)
        self.summary_wx_desc_label = qtw.QLabel()
        self.summary_wx_desc_label.setAccessibleName("wx_desc")
        self.summary_wx_desc_label.setObjectName("wx_desc")      
        font = self.summary_wx_desc_label.font()
        font.setPointSize(20)
        self.summary_wx_desc_label.setFont(font)
        self.summary_wx_desc_label.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        self.summary_wx_desc_label.setFrameStyle(self.frame_style)  
        layout.addWidget(self.summary_wx_desc_label)
           
    def setup_frame_home_restart_system(self,f):
        f['title_label'].setText("Restart ROS Home System?")
           
    def setup_frame_lighting_view(self,f):
        f['title_label'].setText("All Lights")

    def setup_keyboard_shortcuts(self):
        self.shorty_quit_c = qtg.QShortcut(qtg.QKeySequence('Ctrl+C'), self)
        self.shorty_quit_c.activated.connect(self.app.quit)

        self.shorty_quit_q = qtg.QShortcut(qtg.QKeySequence('Ctrl+Q'), self)
        self.shorty_quit_q.activated.connect(self.app.quit)
        
        self.shorty_next_page = qtg.QShortcut(qtg.QKeySequence('Tab'), self)
        self.shorty_next_page.activated.connect(self.next_page)

        self.shorty_prev_page = qtg.QShortcut(qtg.QKeySequence('Shift+Tab'), self)
        self.shorty_prev_page.activated.connect(self.prev_page)
       
        self.shorty_home = qtg.QShortcut(qtg.QKeySequence('Home'), self)
        self.shorty_home.activated.connect(self.home_page)

        self.shorty_full = qtg.QShortcut(qtg.QKeySequence('Esc'), self)
        self.shorty_full.activated.connect(self.max_min)
       
    def next_page(self):
        count = self.stack.count()
        current = self.stack.currentIndex()   
        current += 1
        if current >= count:
            current = 0
        self.stack.setCurrentIndex(current)   
                
    def prev_page(self):
        count = self.stack.count()
        current = self.stack.currentIndex()   
        current -= 1
        if current < 0:
            current = count-1
        self.stack.setCurrentIndex(current)      
        
    def home_page(self):
        self.stack.setCurrentIndex(0)   
        
    def max_min(self):          
        if self.isFullScreen():
            # self.setWindowFlags(self._flags)
            self.showNormal()
        else:
            self._flags = self.windowFlags()
            # self.setWindowFlags(qtc.Qt.WindowCloseButtonHint | Qt.WindowType_Mask)
            self.showFullScreen() 
            
################################################
#
#  Start the QT app, which starts the ROS node
#
################################################        
def main(args=None):
    rclpy.init(args=args)
    home_ui = RQTHomeUI()
    home_ui.show()
    ret=home_ui.app.exec()
    rclpy.shutdown()
    home_ui.ros_node.destroy_node()
    sys.exit(ret)

if __name__ == '__main__':
    main()        

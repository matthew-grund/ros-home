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
import urllib.request
from types import NoneType

import PySide6.QtWidgets as qtw
import PySide6.QtCore as qtc
import PySide6.QtGui as qtg

from . import ui_node
from . import central_widget
from . import qt_left_toolbar    
from . import rqt_style_sheet
from . import keyboard_shortcuts        
              
#######################################################################
#
#     The class ROSHomeUI is the main class in this app. It owns a ROS
#     node that gets spin_once() from a QT timer.
#
class RQTHomeUI(qtw.QMainWindow):
    
    def __init__(self):
        self.app = qtw.QApplication(sys.argv)
        qtw.QMainWindow.__init__(self) 
        self.title = "ROS Home"
        self.description = "Interface to ROS Home"
        self.version_str = "0.7.3" 
        self.copyright_str = "(c) copyright 2022, Matthew Grund"
        self.screen = self.app.primaryScreen()
        self.screen_size =  self.screen.size()
        self.setWindowTitle(self.title)
        self.resize(int(self.screen_size.width()*0.80),int(self.screen_size.height()*0.80))
        
        self.last_wx_temp_deg_f = 99 
        self.stacked_frame_dict = {}   # a dict of categories - each is a list of frame names
        self.last_art_url = ""
        self.last_art_url_2 = ""
        self.renderer_dict = {}
        self.overview_icon_width = 228

        self.app.setStyleSheet(rqt_style_sheet.qss)
        self.frame_style = qtw.QFrame.Shape.Panel  # .Panel for designing, .NoFrame for a clean look    
        self.big_clock = self.styled_label(48)
        self.big_date  = self.styled_label(20)
 
        self.max_nodes_running = 0
               
        central_widget.configure(self)
        central_widget.setup(self)
        self.setup_left_toolbar()
        keyboard_shortcuts.setup(self)
        
        # show the version, to start
        self.statusBar().showMessage(self.title + "      ver:" + self.version_str + "       " + self.copyright_str)
        
        self.ros_node = ui_node.ROSHomeUINode()
        self.ros_spin_timer = qtc.QTimer()
        self.ros_spin_timer.singleShot(50, self.ros_timer_callback)
        self.ros_spin_count = 0
        self.clock_tick_timer = qtc.QTimer()
        self.clock_tick_timer.singleShot(200,self.ui_clock_timer_callback)
        
        self.show()
        
    def ros_timer_callback(self):
        # do ros stuff, then check for ros results
        self.lighting_msg_count = self.ros_node.lighting_msg_count
        self.event_msg_count = self.ros_node.event_msg_count
        self.conditions_msg_count = self.ros_node.conditions_msg_count        
        self.devices_msg_count = self.ros_node.devices_msg_count
        self.nodes_msg_count = self.ros_node.nodes_msg_count
        self.scene_msg_count = self.ros_node.scene_msg_count
        self.media_msg_count = self.ros_node.media_msg_count
        rclpy.spin_once(self.ros_node,timeout_sec=0.075) 
        self.ros_node.spin_count += 1 
        self.check_for_ros_msgs()
        self.ros_spin_timer.singleShot(50,self.ros_timer_callback)
    
    def ui_clock_timer_callback(self):
        now = datetime.now()
        self.clock_str = now.strftime("%H:%M:%S")
        self.date_str = now.strftime("%A, %B %-d, %Y")
        self.big_clock.setText(self.clock_str)
        self.big_date.setText(self.date_str)
        self.clock_tick_timer.singleShot(300,self.ui_clock_timer_callback)
    
    def check_for_ros_msgs(self):
        if self.lighting_msg_count < self.ros_node.lighting_msg_count:          # lighting msg
            self.lighting_msg_count = self.ros_node.lighting_msg_count
            self.ui_parse_lighting_msg(self.ros_node.latest_lighting_msg)
            
        if self.event_msg_count < self.ros_node.event_msg_count:                # event msg
            self.event_msg_count = self.ros_node.event_msg_count
            self.ui_parse_event_msg(self.ros_node.latest_event_msg)
            
        if self.conditions_msg_count < self.ros_node.conditions_msg_count:      # weather conditions msg
            self.conditions_msg_count = self.ros_node.conditions_msg_count
            self.ui_parse_conditions_msg(self.ros_node.latest_conditions_msg)
                
        if self.devices_msg_count < self.ros_node.devices_msg_count:            # known devices msg
            self.devices_msg_count = self.ros_node.devices_msg_count
            self.ui_parse_devices_msg(self.ros_node.latest_devices_msg)
                
        if self.nodes_msg_count < self.ros_node.nodes_msg_count:                # nodes msg
            self.nodes_msg_count = self.ros_node.nodes_msg_count
            self.ui_parse_nodes_msg(self.ros_node.latest_nodes_msg)
                
        if self.scene_msg_count < self.ros_node.scene_msg_count:                # lighting scenes msg
            self.scene_msg_count = self.ros_node.scene_msg_count
            self.ui_parse_scene_msg(self.ros_node.latest_scene_msg)
             
        if self.media_msg_count < self.ros_node.media_msg_count:                # media msg  
            self.media_msg_count = self.ros_node.media_msg_count
            self.ui_parse_media_msg(self.ros_node.latest_media_msg) 
                        
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
 
    def ui_parse_scene_msg(self,msg):
        self.statusBar().showMessage(f"Got lighting scene update for {msg['payload']['today']}: {msg['payload']['previous_scene']}")
        self.current_scene_label.setText(f"{msg['payload']['previous_scene'].replace('_',' ')}")
        self.next_scene_label.setText(f"{msg['payload']['next_scene'].replace('_',' ')}")  
        hrs_remain  = int( 10.0 *msg['payload']['secs_remaining']/3600)/10.0            
        self.next_scene_time_label.setText(f"{hrs_remain} hrs")           
           
    def ui_parse_conditions_msg(self,msg):
        icon_url = msg['payload']['icon']
        if type(icon_url) != NoneType and len(icon_url) :
            try:
                data = urllib.request.urlopen(icon_url).read()
            except:
                return    
            image = qtg.QImage()
            image.loadFromData(data)
            scaled_image = image.scaledToWidth(self.overview_icon_width)
            self.summary_wx_cond_icon.setPixmap(qtg.QPixmap(scaled_image))
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
        num_nodes = len(msg['payload'])
        if self.max_nodes_running < num_nodes:
            self.max_nodes_running = num_nodes
        self.nodes_summary_label.setText(f"{ts}   ROS Home: {num_nodes} ROS nodes currently running. (max is {self.max_nodes_running} nodes running).")
    
    def ui_parse_media_msg(self,msg):
        # self.statusBar().showMessage(f"Got renderer status from {msg['payload']['name']} ({msg['payload']['vendor']} {msg['payload']['type']})")
        self.update_media_renderer_dict(msg)
        if self.best_renderer['status']['power'] == 'on': 
                art_url = self.best_renderer['play']['albumart_url']
                if len(art_url):
                    # print(art_url)
                    self.last_art_url = art_url
                    full_url = "http://" + self.best_renderer['ip'] + art_url
                    data = urllib.request.urlopen(full_url).read()
                    image = qtg.QImage()
                    image.loadFromData(data)
                    scaled_image = image.scaledToWidth(self.overview_icon_width)
                    self.album_art_label.setPixmap(qtg.QPixmap(scaled_image))
                    song = self.best_renderer['play']['track']
                    if len(song) > 27:
                        song = song[:25] +'...'       
                    self.playback_song_label.setText(song)
                    album = self.best_renderer['play']['album']
                    if len(album) > 36:
                        album = album[:34] +'...'
                    self.playback_album_label.setText(album)
                    self.playback_artist_label.setText(self.best_renderer['play']['artist'])
                else: 
                        print("no  album art for ")
                        self.last_art_url = ""   
                        self.playback_song_label.setText(self.best_renderer['name'] + ' ON')
                        self.playback_album_label.setText('input is '+self.best_renderer['status']['input'])
                        self.playback_artist_label.setText('')
                        data = urllib.request.urlopen(self.best_renderer['device_icon_url']).read()
                        image = qtg.QImage()
                        image.loadFromData(data)
                        scaled_image = image.scaledToWidth(self.overview_icon_width)
                        self.album_art_label.setPixmap(qtg.QPixmap(scaled_image))
        else:
            self.last_art_url = ""
            self.playback_song_label.setText(self.best_renderer['name'] + ' OFF')
            self.playback_album_label.setText('')
            self.playback_artist_label.setText('')
            data = urllib.request.urlopen(self.best_renderer['device_icon_url']).read()
            image = qtg.QImage()
            image.loadFromData(data)
            scaled_image = image.scaledToWidth(self.overview_icon_width)
            self.album_art_label.setPixmap(qtg.QPixmap(scaled_image))                
                
    def update_media_renderer_dict(self,msg):
            self.renderer_dict[msg['payload']['ip']] = msg['payload']
            best = self.find_most_active('')
            if len(best):
               self.best_renderer = self.renderer_dict[best]
            else:
                self.best_renderer = {}  
            best_2 =  self.find_most_active(best)
            if len(best_2):
               self.best_renderer_2 = self.renderer_dict[best_2]
            else:
                self.best_renderer_2 = {}      
               
    def find_most_active(self,skip_ip):
        max_score = 0
        max_ip = ''
        for ip in self.renderer_dict:
            if ip != skip_ip:
                score = 0
                r = self.renderer_dict[ip]
                if r['status']['power'] == "on":
                    score += 300
                    if r['play']['playback'] == "play":
                       score += 200
                       if r['status']['mute'] == False:
                           score += r['status']['volume'] / r['status']['max_volume'] * 100
                if score >= max_score:
                    max_score = score
                    max_ip = ip    
        return max_ip
    
    def menu_callback(self,parent_name, action_name):   
        print("Menu" + parent_name + ":" + action_name)
        self.statusBar().showMessage(parent_name + ":" + action_name)
        self.stack.setCurrentIndex(self.frame_dict[parent_name][action_name]['index'])

    def setup_left_toolbar(self):
        self.left_toolbar = qt_left_toolbar.QTLeftToolBar(self)
        
    def styled_label(self,fontsize): 
        styled_label = qtw.QLabel()
        font = styled_label.font()
        font.setPointSize(fontsize)
        styled_label.setFont(font)    
        styled_label.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        styled_label.setFrameStyle(self.frame_style)  
        return styled_label   
           
    def setup_frame_home_restart_system(self, panel):
        panel.title_label.setText("Restart ROS Home System?")
           
    def setup_frame_lighting_view(self,panel):
        panel.title_label.setText("All Lights") 
                
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

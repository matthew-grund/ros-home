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

import PyQt6.QtWidgets as qtw
import PyQt6.QtCore as qtc
import PyQt6.QtGui as qtg

class RQTCentralFrame(qtw.QFrame):
    def __init__(self,parent, menu_name, item_name, frame_style):
        super().__init__()

        # frame 
        self.frame = qtw.QFrame(parent)
        frame_name_str = menu_name + "_" + item_name + "_frame"
        self.frame.setAccessibleName(frame_name_str)
        self.frame.setObjectName(frame_name_str)
        self.frame.setFrameStyle(frame_style)
        # layout
        vert_layout = qtw.QVBoxLayout(self.frame)
        vert_layout_name_str = menu_name + "_" + item_name + "_layout"
        vert_layout.setObjectName(vert_layout_name_str)
        vert_layout.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        vert_layout.setSpacing(8)
        if self.num_frames == 0:
            vert_layout.addWidget(self.big_clock)
            vert_layout.addWidget(self.big_date)
            vert_layout.addWidget(self.styled_label(18))
        else:    
            title_label = self.styled_label(18)
            title_label.setText(menu_name + " : " + item_name )
            vert_layout.addWidget(title_label)
        main_horiz_layout = qtw.QHBoxLayout()
        lh_panel = qtw.QFrame()

        lh_panel.setFrameStyle(self.frame_style)
        lh_panel.setObjectName(menu_name + "_" + item_name + "_lh_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("lh_panel")        
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        lh_panel.setLayout(layout)   
        center_panel = qtw.QFrame()

        center_panel.setFrameStyle(self.frame_style)
        center_panel.setObjectName(menu_name + "_" + item_name + "_center_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("center_panel")        
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        center_panel.setLayout(layout)
           
        rh_panel = qtw.QFrame()
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
        if 'view' in item_name:
            self.view_list.append(self.num_frames)
        self.num_frames += 1
        custom_method_name = "setup_frame_" + menu_name.replace(' ','_') + "_" + item_name.replace(' ','_')
        if hasattr(self, custom_method_name):
            customize = getattr(self,custom_method_name)
            if callable(customize):
                # print(f"found customization method: {custom_method_name}")
                customize(self.frame_dict[menu_name][item_name])

        
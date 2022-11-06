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

class QTCentralFrame(qtw.QFrame):
    def __init__(self,parent, menu_name, item_name, frame_style):
        super().__init__()

        self.frame_style = frame_style
        # frame 
        self.frame = qtw.QFrame(parent)
        frame_name_str = menu_name + "_" + item_name + "_frame"
        self.frame.setAccessibleName(frame_name_str)
        self.frame.setObjectName(frame_name_str)
        self.frame.setFrameStyle(frame_style)
        # layout
        self.vert_layout = qtw.QVBoxLayout(self.frame)
        vert_layout_name_str = menu_name + "_" + item_name + "_layout"
        self.vert_layout.setObjectName(vert_layout_name_str)
        self.vert_layout.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        self.vert_layout.setSpacing(8)  
        self.title_label = self.styled_label(18)
        self.title_label.setText(menu_name + " : " + item_name )
        self.vert_layout.addWidget(self.title_label)
        self.main_horiz_layout = qtw.QHBoxLayout()
        # lh panel
        self.lh_panel = qtw.QFrame()
        self.lh_panel.setFrameStyle(frame_style)
        self.lh_panel.setObjectName(menu_name + "_" + item_name + "_lh_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("lh_panel")        
        label.setFrameStyle(frame_style)
        layout.addWidget(label)
        self.lh_panel.setLayout(layout)   
        self.center_panel = qtw.QFrame()
        self.center_panel.setFrameStyle(self.frame_style)
        self.center_panel.setObjectName(menu_name + "_" + item_name + "_center_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("center_panel")        
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        self.center_panel.setLayout(layout)
        # rh panel
        self.rh_panel = qtw.QFrame()
        self.rh_panel.setFrameStyle(self.frame_style)
        self.rh_panel.setObjectName(menu_name + "_" + item_name + "_rh_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("rh_panel")        
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        self.rh_panel.setLayout(layout)   
        self.main_horiz_layout.addWidget(self.lh_panel)
        self.main_horiz_layout.addWidget(self.center_panel)
        self.main_horiz_layout.addWidget(self.rh_panel)
        self.main_horiz_layout.setSpacing(128)
        # footer panel
        self.footer_panel = qtw.QFrame()
        self.footer_panel.setFrameStyle(self.frame_style)
        self.footer_panel.setObjectName(menu_name + "_" + item_name + "_footer_panel")
        layout = qtw.QVBoxLayout()
        label = qtw.QLabel("footer_panel")
        label.setFrameStyle(self.frame_style)
        layout.addWidget(label)
        self.footer_panel.setLayout(layout)    
           
        # bring it on home
        self.vert_layout.addLayout(self.main_horiz_layout)
        self.vert_layout.addWidget(self.footer_panel)
        self.frame.setLayout(self.vert_layout)
        

    def styled_label(self,fontsize): 
        styled_label = qtw.QLabel()
        font = styled_label.font()
        font.setPointSize(fontsize)
        styled_label.setFont(font)    
        styled_label.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
        styled_label.setFrameStyle(self.frame_style)  
        return styled_label   
               
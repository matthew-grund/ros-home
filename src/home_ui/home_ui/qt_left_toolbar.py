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


class QTLeftToolBar(qtw.QToolBar):

    def __init__(self, q_main_window):
        super().__init__()
        self.q_main_window = q_main_window
        self.toolbar = qtw.QToolBar("Left toolbar")
        self.toolbar.setIconSize(qtc.QSize(56,56))
        self.toolbar.setMovable(False)
        self.icon_path = "/data/home_ws/icons/"
        q_main_window.addToolBar(qtc.Qt.LeftToolBarArea,self.toolbar)
        
        self.add_toolbar_spacer()
        self.add_orange_toolbar_button("home","overview","home_filled_96.png")
        self.toolbar.addSeparator()
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("people","view","group_96.png")
        self.toolbar.addSeparator()
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("lighting","view","lightbulb_96.png")
        self.toolbar.addSeparator()
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("temperature","view","thermostat_96.png")
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("security","view","security_96.png")
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("media","view","music_note_96.png")
        self.toolbar.addSeparator()
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("nodes","view","apps_96.png")
        self.toolbar.addSeparator()
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("diagnostics","events","tune_96.png")
        self.toolbar.addSeparator()
        self.toolbar.addSeparator()
        # self.add_toolbar_spacer()
        self.add_white_toolbar_button("help","about","help_outline_96.png")
        self.add_toolbar_spacer()
        
    def add_toolbar_spacer(self):
        spacer = qtw.QWidget()
        spacer.setSizePolicy(qtw.QSizePolicy.Expanding, qtw.QSizePolicy.Expanding)
        self.toolbar.addWidget(spacer)
        
    def add_white_toolbar_button(self,menu_name,item_name,icon_file):
        i = self.white_icon(self.icon_path + icon_file)
        a = qtg.QAction(i, menu_name.title(), self)
        a.triggered.connect(lambda : self.q_main_window.toolbar_callback(menu_name,item_name))
        self.toolbar.addAction(a)

    def add_orange_toolbar_button(self,menu_name,item_name,icon_file):
        i = self.orange_icon(self.icon_path + icon_file)
        a = qtg.QAction(i, menu_name.title(), self)
        a.triggered.connect(lambda : self.q_main_window.toolbar_callback(menu_name,item_name))
        self.toolbar.addAction(a)
        
    def color_icon(self, filename, color):
        pixmap = qtg.QPixmap(filename)
        painter = qtg.QPainter()
        painter.begin(pixmap)
        painter.setCompositionMode(qtg.QPainter.CompositionMode_SourceIn)
        painter.fillRect(pixmap.rect(), qtg.QColor(color))
        painter.end()
        icon = qtg.QIcon(pixmap)
        return icon

    def orange_icon(self,filename):
        return self.color_icon(filename,"#ffb75b")
        
    def white_icon(self,filename):
        return self.color_icon(filename,"white")
    
    def toolbar_callback(self, parent_name, action_name):   
        print("Toolbar: " + parent_name + ":" + action_name)
        self.q_main_window.statusBar().showMessage(parent_name + ":" + action_name)
        self.q_main_window.stacked_layout.setCurrentIndex(self.q_main_window.stacked_frame_indices[parent_name][action_name])   
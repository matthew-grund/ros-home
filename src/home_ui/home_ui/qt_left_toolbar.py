#
#  ROS Home Qt UI
#
# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#

import PySide6.QtWidgets as qtw
import PySide6.QtCore as qtc
import PySide6.QtGui as qtg

import rqt_home_widgets as rhw
class QTLeftToolBar(qtw.QToolBar):

    def __init__(self, q_main_window):
        super().__init__()
        self.q_main_window = q_main_window
        self.setIconSize(qtc.QSize(32,32))
        self.setMovable(False)
        self.icon_path = "/data/home_ws/icons/"
        q_main_window.addToolBar(qtc.Qt.LeftToolBarArea,self)
        self.current_active_button_group = "home"
        self.current_active_button_item = "overview"
        self.active_button_color = "#ffb75b"
        self.normal_button_color = "white"
        self.action_dict = {}
    
        # layout the buttons in the toolbar    
        self.add_toolbar_spacer()
        self.add_toolbar_spacer()
        self.add_active_toolbar_button("home","overview","home_filled_96.png")  # these names correspond to frame names in central_widget.py
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("people","view","group_96.png")
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("lighting","view","lightbulb_96.png")
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("temperature","view","thermostat_96.png")
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("security","view","security_96.png")
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("media","view","music_note_96.png")
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("nodes","view","apps_96.png")
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("diagnostics","events","tune_96.png")
        self.add_toolbar_spacer()
        self.add_normal_toolbar_button("help","about","help_outline_96.png")
        self.add_toolbar_spacer()
        self.add_toolbar_spacer()
        
    def add_toolbar_spacer(self):
        spacer = rhw.styled_spacer(self.q_main_window)
        self.addWidget(spacer)
        
    def add_normal_toolbar_button(self,group_name,item_name,icon_file):
        i = self.normal_icon(self.icon_path + icon_file)
        a = qtg.QAction(i, group_name.title(), self)
        a.triggered.connect(lambda : self.toolbar_callback(group_name,item_name))
        self.addAction(a)
        if group_name not in self.action_dict:
            self.action_dict[group_name] = {}
        self.action_dict[group_name][item_name] = a  

    def add_active_toolbar_button(self,group_name,item_name,icon_file):
        i = self.active_icon(self.icon_path + icon_file)
        a = qtg.QAction(i, group_name.title(), self)
        a.triggered.connect(lambda : self.toolbar_callback(group_name,item_name))
        self.addAction(a)
        if group_name not in self.action_dict:
            self.action_dict[group_name] = {}
        self.action_dict[group_name][item_name] = a  
        
    def color_icon(self, filename, color):
        pixmap = qtg.QPixmap(filename)
        painter = qtg.QPainter()
        painter.begin(pixmap)
        painter.setCompositionMode(qtg.QPainter.CompositionMode_SourceIn)
        painter.fillRect(pixmap.rect(), qtg.QColor(color))
        painter.end()
        icon = qtg.QIcon(pixmap)
        return icon

    def active_icon(self,filename):
        return self.color_icon(filename,"#ffb75b")
        
    def normal_icon(self,filename):
        return self.color_icon(filename,"white")
    
    def toolbar_callback(self, group_name, item_name):   
        # print("Toolbar: " + group_name + ":" + item_name)
        self.select_button(group_name, item_name)
        self.q_main_window.statusBar().showMessage(group_name + ":" + item_name)
        self.q_main_window.stacked_layout.setCurrentIndex(self.q_main_window.stacked_frame_indices[group_name][item_name]) 
    
    def select_button(self, group_name, item_name):
        self.colorize_button(group_name,item_name,self.active_button_color)
        self.colorize_button(self.current_active_button_group, self.current_active_button_item, self.normal_button_color)
        self.current_active_button_group = group_name
        self.current_active_button_item = item_name  
          
    def colorize_button(self, group_name, item_name, color):
        if group_name in self.action_dict:
            if item_name not in self.action_dict[group_name]:
                item_name = next(iter(self.action_dict[group_name]))
            a = self.action_dict[group_name][item_name]
            i = a.icon()
            pixy = i.pixmap(96)         
            painter = qtg.QPainter()
            painter.begin(pixy)
            painter.setCompositionMode(qtg.QPainter.CompositionMode_SourceIn)
            painter.fillRect(pixy.rect(), qtg.QColor(color))
            painter.end()
            icon = qtg.QIcon(pixy)
        a.setIcon(icon)
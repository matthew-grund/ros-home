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

def create_home_overview_frame(qt_main_window):
    print("in create_home_overview function")
    frame = qtw.QFrame()
    frame_name = "create_home_overview_frame (create)"
    frame.setObjectName(frame_name)
    frame.setAccessibleName(frame_name)
    frame.setFrameStyle(qt_main_window.frame_style)
    v_layout = qtw.QVBoxLayout()
    title = styled_label(qt_main_window,24)
    title.setText(frame_name)
    v_layout.addWidget(title)
    frame.setLayout(v_layout)
    
    return frame

def styled_label(qt_main_window,fontsize): 
    styled_label = qtw.QLabel()
    font = styled_label.font()
    font.setPointSize(fontsize)
    styled_label.setFont(font)    
    styled_label.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
    styled_label.setFrameStyle(qt_main_window.frame_style)  
    return styled_label  
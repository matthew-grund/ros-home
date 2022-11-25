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

def styled_frame(qt_main_window, name):
    styled_frame = qtw.QFrame()
    styled_frame.setObjectName(name)
    styled_frame.setAccessibleName(name)
    styled_frame.setFrameStyle(qt_main_window.frame_style)
    return styled_frame

def styled_spacer(qt_main_window):
    spacer = qtw.QFrame()
    spacer.setFrameStyle(qt_main_window.frame_style)  
    spacer.setSizePolicy(qtw.QSizePolicy.Expanding, qtw.QSizePolicy.Expanding)

def styled_label(qt_main_window, text, fontsize): 
    styled_label = qtw.QLabel()
    font = styled_label.font()
    font.setPointSize(fontsize)
    styled_label.setFont(font)    
    styled_label.setText(text)
    styled_label.setAlignment(qtc.Qt.AlignmentFlag.AlignCenter | qtc.Qt.AlignmentFlag.AlignVCenter)
    styled_label.setFrameStyle(qt_main_window.frame_style)  
    return styled_label
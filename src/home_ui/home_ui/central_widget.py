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

from . import home_frames
from . import people_frames
from . import lighting_frames
from . import temperature_frames
from . import security_frames
from . import media_frames
from . import nodes_frames
from . import diagnostics_frames

def configure(qt_main_window):
    # every stacked frame in the central widget
    qt_main_window.stacked_frame_dict = {}   # a dict of lists of frames. item [0] of each frame list is attached to the tool bar button
    # home menu
    qt_main_window.stacked_frame_dict["home"] = []
    qt_main_window.stacked_frame_dict["home"].append("overview")
    qt_main_window.stacked_frame_dict["home"].append("location")
    qt_main_window.stacked_frame_dict["home"].append("restart system")
    # people menu
    qt_main_window.stacked_frame_dict["people"] = []
    qt_main_window.stacked_frame_dict["people"].append("view")
    qt_main_window.stacked_frame_dict["people"].append("add")       
    qt_main_window.stacked_frame_dict["people"].append("track")
        # lighting menu
    qt_main_window.stacked_frame_dict["lighting"] = []
    qt_main_window.stacked_frame_dict["lighting"].append("view")
    qt_main_window.stacked_frame_dict["lighting"].append("scenes")
    qt_main_window.stacked_frame_dict["lighting"].append("schedule")
    qt_main_window.stacked_frame_dict["lighting"].append("triggers")
    # temperature menu
    qt_main_window.stacked_frame_dict["temperature"] = []
    qt_main_window.stacked_frame_dict["temperature"].append("view")
    qt_main_window.stacked_frame_dict["temperature"].append("modes")
    qt_main_window.stacked_frame_dict["temperature"].append("schedule")
    qt_main_window.stacked_frame_dict["temperature"].append("triggers")
    # security menu
    qt_main_window.stacked_frame_dict["security"] = []
    qt_main_window.stacked_frame_dict["security"].append("view")
    qt_main_window.stacked_frame_dict["security"].append("arm")
    qt_main_window.stacked_frame_dict["security"].append("disarm")
    qt_main_window.stacked_frame_dict["security"].append("schedule")
    # media menu
    qt_main_window.stacked_frame_dict["media"] = []
    qt_main_window.stacked_frame_dict["media"].append("view")
    qt_main_window.stacked_frame_dict["media"].append("stagings")
    qt_main_window.stacked_frame_dict["media"].append("schedule")
    qt_main_window.stacked_frame_dict["media"].append("triggers")
    # nodes
    qt_main_window.stacked_frame_dict["nodes"] = []
    qt_main_window.stacked_frame_dict["nodes"].append("view")
    qt_main_window.stacked_frame_dict["nodes"].append("messages")
    qt_main_window.stacked_frame_dict["nodes"].append("run")        
    # devices
    qt_main_window.stacked_frame_dict["devices"] = []
    qt_main_window.stacked_frame_dict["devices"].append("view")
    qt_main_window.stacked_frame_dict["devices"].append("messages")
    qt_main_window.stacked_frame_dict["devices"].append("all known")
    qt_main_window.stacked_frame_dict["devices"].append("identify")     
    # diagnostics
    qt_main_window.stacked_frame_dict["diagnostics"] = []        
    qt_main_window.stacked_frame_dict["diagnostics"].append("events")
    qt_main_window.stacked_frame_dict["diagnostics"].append("measurements")        
    qt_main_window.stacked_frame_dict["diagnostics"].append("ros console")
    # help
    qt_main_window.stacked_frame_dict["help"] = []        
    qt_main_window.stacked_frame_dict["help"].append("about")
    qt_main_window.stacked_frame_dict["help"].append("getting started")

def setup(qt_main_window):
    index = 0
    qt_main_window.central_widget = qtw.QWidget()
    qt_main_window.stacked_layout = qtw.QStackedLayout()
    
    qt_main_window.stacked_frame_indices = {}
    for group in qt_main_window.stacked_frame_dict:
        qt_main_window.stacked_frame_indices[group] = {}
        for item in qt_main_window.stacked_frame_dict[group]:
            qt_main_window.stacked_frame_indices[group][item] = index
            index += 1
            namespace = globals()
            module_name = group + "_frames"
            if module_name in namespace:
                method_name = "create_" + group + "_" + item + "_frame"
                module_obj = namespace[module_name]
                if hasattr(module_obj,method_name) and callable(getattr(module_obj,method_name)):
                    create = getattr(module_obj,method_name)
                    frame = create(qt_main_window)
                else:    
                    frame = placeholder_frame(qt_main_window,group,item)
            else:
                frame = placeholder_frame(qt_main_window,group,item)
            qt_main_window.stacked_layout.addWidget(frame)
    qt_main_window.central_widget.setLayout(qt_main_window.stacked_layout)
    qt_main_window.setCentralWidget(qt_main_window.central_widget)    

def placeholder_frame(qt_main_window,group,item):
    frame = qtw.QFrame()
    frame_name = group + "_" + item + "_frame"
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
        
def next_page(qt_main_window):
    count = qt_main_window.stacked_layout.count()
    current = qt_main_window.stacked_layout.currentIndex()   
    current += 1
    if current >= count:
        current = 0
    qt_main_window.stacked_layout.setCurrentIndex(current)  
    
def prev_page(qt_main_window):
    count = qt_main_window.stacked_layout.count()
    current = qt_main_window.stacked_layout.currentIndex()   
    current -= 1
    if current < 0:
        current = count-1
    qt_main_window.stacked_layout.setCurrentIndex(current)  
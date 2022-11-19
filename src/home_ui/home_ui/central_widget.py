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
    qt_main_window.stacked_frame_dict["nodes"].append("run...")        
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
            frame = qtw.QFrame()
            frame_name = group + "_" + item + "_frame"
            frame.setObjectName(frame_name)
            frame.setAccessibleName(frame_name)
            frame.setFrameStyle(qt_main_window.frame_style)
            v_layout = qtw.QVBoxLayout()
            title = qtw.QLabel()
            title.setText(frame_name)
            v_layout.addWidget(title)
            frame.setLayout(v_layout)
    qt_main_window.setLayout(qt_main_window.stacked_layout)
        
        
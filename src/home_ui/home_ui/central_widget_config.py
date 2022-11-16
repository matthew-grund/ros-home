#
#  ROS Home Qt UI
#
# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#

def setup(qt_main_window):
    # every stacked frame in the central widget
        qt_main_window.stacked_frame_dict = {}
        # home menu
        qt_main_window.stacked_frame_list["home"] = []
        qt_main_window.stacked_frame_list["home"].append("overview")
        qt_main_window.stacked_frame_list["home"].append("location")
        qt_main_window.stacked_frame_list["home"].append("restart system")
        # people menu
        qt_main_window.stacked_frame_list["people"] = []
        qt_main_window.stacked_frame_list["people"].append("view")
        qt_main_window.stacked_frame_list["people"].append("add")       
        qt_main_window.stacked_frame_list["people"].append("track")
        # lighting menu
        qt_main_window.stacked_frame_list["lighting"] = []
        qt_main_window.stacked_frame_list["lighting"].append("view")
        qt_main_window.stacked_frame_list["lighting"].append("scenes")
        qt_main_window.stacked_frame_list["lighting"].append("schedule")
        qt_main_window.stacked_frame_list["lighting"].append("triggers")
        # temperature menu
        qt_main_window.stacked_frame_list["temperature"] = []
        qt_main_window.stacked_frame_list["temperature"].append("view")
        qt_main_window.stacked_frame_list["temperature"].append("modes")
        qt_main_window.stacked_frame_list["temperature"].append("schedule")
        qt_main_window.stacked_frame_list["temperature"].append("triggers")
        # security menu
        qt_main_window.stacked_frame_list["security"] = []
        qt_main_window.stacked_frame_list["security"].append("view")
        qt_main_window.stacked_frame_list["security"].append("arm")
        qt_main_window.stacked_frame_list["security"].append("disarm")
        qt_main_window.stacked_frame_list["security"].append("schedule")
        # media menu
        qt_main_window.stacked_frame_list["media"] = []
        qt_main_window.stacked_frame_list["media"].append("view")
        qt_main_window.stacked_frame_list["media"].append("stagings")
        qt_main_window.stacked_frame_list["media"].append("schedule")
        qt_main_window.stacked_frame_list["media"].append("triggers")
        # nodes
        qt_main_window.stacked_frame_list["nodes"] = []
        qt_main_window.stacked_frame_list["nodes"].append("view")
        qt_main_window.stacked_frame_list["nodes"].append("messages")
        qt_main_window.stacked_frame_list["nodes"].append("run...")        
        # devices
        qt_main_window.stacked_frame_list["devices"] = []
        qt_main_window.stacked_frame_list["devices"].append("view")
        qt_main_window.stacked_frame_list["devices"].append("messages")
        qt_main_window.stacked_frame_list["devices"].append("all known")
        qt_main_window.stacked_frame_list["devices"].append("identify")     
        # diagnostics
        qt_main_window.stacked_frame_list["diagnostics"] = []        
        qt_main_window.stacked_frame_list["diagnostics"].append("events")
        qt_main_window.stacked_frame_list["diagnostics"].append("measurements")        
        qt_main_window.stacked_frame_list["diagnostics"].append("ros console")
        # help
        qt_main_window.stacked_frame_list["help"] = []        
        qt_main_window.stacked_frame_list["help"].append("about")
        qt_main_window.stacked_frame_list["help"].append("getting started")

    
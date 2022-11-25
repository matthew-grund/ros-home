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

from . import rqt_home_widgets as rhw

def create_home_overview_frame(qt_main_window):
    side_panel_width = 320 
    frame = rhw.styled_frame(qt_main_window,"create_home_overview_frame main frame")
    v_layout = qtw.QVBoxLayout()

    # clock
    v_layout.addWidget(qt_main_window.big_clock)
    # date
    v_layout.addWidget(qt_main_window.big_date)  

    dummy_label = rhw.styled_label(qt_main_window,"",16)
    v_layout.addWidget(dummy_label)    
    # middle "content" box
    mh_frame = rhw.styled_frame(qt_main_window,"create_home_overview_frame middle frame")
    v_layout.addWidget(mh_frame)
    mh_layout = qtw.QHBoxLayout()

    # the left hand body pane: weather summary
    lh_frame = rhw.styled_frame(qt_main_window,"create_home_overview_frame left panel")
    mh_layout.addWidget(lh_frame)
    lh_frame.setMinimumWidth(side_panel_width)
    lh_layout = qtw.QVBoxLayout()
    qt_main_window.summary_wx_cond_icon =  rhw.styled_label(qt_main_window,"", 48)   
    lh_layout.addWidget(qt_main_window.summary_wx_cond_icon)
    qt_main_window.summary_wx_temp_label = rhw.styled_label(qt_main_window,"temp", 64)   
    lh_layout.addWidget(qt_main_window.summary_wx_temp_label)
    qt_main_window.summary_wx_desc_label = rhw.styled_label(qt_main_window,"desc", 32) 
    lh_layout.addWidget(qt_main_window.summary_wx_desc_label)

    # center body pane: lighting
    cb_frame = rhw.styled_frame(qt_main_window,"create_home_overview_frame center frame")
    mh_layout.addWidget(cb_frame)
    cb_layout = qtw.QVBoxLayout()
    scene_label = rhw.styled_label(qt_main_window,"Current Scene",24)
    cb_layout.addWidget(scene_label)
    qt_main_window.current_scene_label = rhw.styled_label(qt_main_window,"<current>",64) 
    cb_layout.addWidget(qt_main_window.current_scene_label)  
    dummy_label = rhw.styled_label(qt_main_window,"",20)
    cb_layout.addWidget(dummy_label)    
    next_label = rhw.styled_label(qt_main_window, "Next Scene", 24)    
    cb_layout.addWidget(next_label)
    qt_main_window.next_scene_label = rhw.styled_label(qt_main_window,"<next>",32)
    cb_layout.addWidget(qt_main_window.next_scene_label) 
    qt_main_window.next_scene_time_label = rhw.styled_label(qt_main_window,"<time>",24)  
    cb_layout.addWidget(qt_main_window.next_scene_time_label)

    # right body pane: media
    rh_frame = rhw.styled_frame(qt_main_window,"create_home_overview_frame right frame")
    rh_frame.setMinimumWidth(side_panel_width)   
    rh_layout = qtw.QVBoxLayout()
    mh_layout.addWidget(rh_frame)
    qt_main_window.album_art_label = rhw.styled_label(qt_main_window,"",24)
    rh_layout.addWidget(qt_main_window.album_art_label)
    qt_main_window.playback_song_label = rhw.styled_label(qt_main_window,"<song>",18)
    rh_layout.addWidget(qt_main_window.playback_song_label)
    qt_main_window.playback_album_label = rhw.styled_label(qt_main_window,"<album>",14)
    rh_layout.addWidget(qt_main_window.playback_album_label)
    qt_main_window.playback_artist_label = rhw.styled_label(qt_main_window,"<artist>",12)
    rh_layout.addWidget(qt_main_window.playback_artist_label)

    # footer bx: some lines of summary text
    f_frame = rhw.styled_frame(qt_main_window,"create_home_overview_frame footer frame")
    v_layout.addWidget(f_frame)
    f_layout = qtw.QVBoxLayout()
    dummy_label = rhw.styled_label(qt_main_window,"",20)
    f_layout.addWidget(dummy_label) 
    qt_main_window.lighting_summary_label = rhw.styled_label(qt_main_window,"<lighting summary>",14)
    f_layout.addWidget(qt_main_window.lighting_summary_label)
    qt_main_window.nodes_summary_label = rhw.styled_label(qt_main_window,"<nodes summary>",14)
    f_layout.addWidget(qt_main_window.nodes_summary_label)
    qt_main_window.devices_summary_label = rhw.styled_label(qt_main_window,"<devices summary>",14)
    f_layout.addWidget(qt_main_window.devices_summary_label)
    f_frame.setLayout(f_layout)
    lh_frame.setLayout(lh_layout)
    cb_frame.setLayout(cb_layout)
    rh_frame.setLayout(rh_layout)
    mh_frame.setLayout(mh_layout)
    frame.setLayout(v_layout)
    return frame

def setup_frame_home_overview(self,panel):

        # rh_panel
        frame = panel.rh_panel

        layout = frame.layout()
        num_widgets = layout.count()
        if (num_widgets):
            layout.removeWidget(layout.itemAt(num_widgets-1).widget())


 
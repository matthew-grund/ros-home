#
#  ROS Home Qt UI
#
# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#

from . import central_widget

import PySide6.QtGui as qtg

def setup(rqt_main_window):
    rqt_main_window.shorty_quit_c = qtg.QShortcut(qtg.QKeySequence('Ctrl+C'), rqt_main_window)
    rqt_main_window.shorty_quit_c.activated.connect(rqt_main_window.app.quit)

    rqt_main_window.shorty_quit_q = qtg.QShortcut(qtg.QKeySequence('Ctrl+Q'), rqt_main_window)
    rqt_main_window.shorty_quit_q.activated.connect(rqt_main_window.app.quit)
        
    rqt_main_window.shorty_next_page = qtg.QShortcut(qtg.QKeySequence('Tab'), rqt_main_window)
    rqt_main_window.shorty_next_page.activated.connect(lambda: central_widget.next_page(rqt_main_window))

    rqt_main_window.shorty_prev_page = qtg.QShortcut(qtg.QKeySequence('Backspace'), rqt_main_window)
    rqt_main_window.shorty_prev_page.activated.connect(lambda: central_widget.prev_page(rqt_main_window))
       
    rqt_main_window.shorty_home = qtg.QShortcut(qtg.QKeySequence('Home'), rqt_main_window)
    rqt_main_window.shorty_home.activated.connect(rqt_main_window.home_page)

    rqt_main_window.shorty_full = qtg.QShortcut(qtg.QKeySequence('Esc'), rqt_main_window)
    rqt_main_window.shorty_full.activated.connect(rqt_main_window.max_min)
    
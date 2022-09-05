# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import configobj
import json

class HomeConfigurator(Node):

    def __init__(self):
        super().__init__('home_config')
        self.folder = "/data/home_ws/config"
        self.publisher_settings = self.create_publisher(String, 'settings', 10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.files = {}

    def timer_callback(self):
        ls = os.listdir(self.folder)
        for f in ls:
            is_new = True
            is_updated = False
            fpath = self.folder + "/" +  f
            for fname in self.files:
                if fname == f:
                    is_new = False
            mt = os.path.getmtime(fpath)
            if not is_new:
                if self.files[f] < mt:
                    is_updated = True
            if is_new or is_updated:
                settings = self.load_ini_file(fpath)
                self.files[f] = mt
                settings['filepath'] = fpath
                settings['type'] =  (f.rsplit('.',1)[0]).upper()   
                self.send_settings_msg(settings)

    def load_ini_file(self,filepath):
        settings = configobj.ConfigObj(filepath)
        return settings

    def send_settings_msg(self,settings):
        m = {}
        m['index'] = self.i
        m['interval'] = self.timer_period
        m['payload'] = settings
        mstr = json.dumps(m)
        msg = String()
        msg.data = mstr
        self.publisher_settings.publish(msg)
        self.get_logger().info('Config loading "%s" - %s' % (settings['type'],mstr))
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    home_configurator = HomeConfigurator()

    rclpy.spin(home_configurator)

    home_configurator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
from cmath import nan
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import datetime
import time
import json


class HomeScheduler(Node):

    def __init__(self):
        super().__init__('home_scheduler')
        self.publisher_ = self.create_publisher(String, 'scene', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.config_subscription = self.create_subscription(
            String,
            'settings',
            self.config_listener_callback,
            10)
        self.config_subscription
        self.sun_subscription = self.create_subscription(
            String,
            'sun',
            self.sun_listener_callback,
            10)
        self.config_subscription
        self.i = 0
        self.need_config_schedule = True
        self.need_named_events = True

    def timer_callback(self):
        msg = String()
        m={}
        m['today'] = self.today
        msg.data = 'Scene: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def config_listener_callback(self,msg):
        if self.need_sun_events == True:
            return
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "SCHEDULE":
            raw_schedule = msg['payload']
            self.schedule = self.process_raw_schedule(raw_schedule)
            self.need_config_schedule = False
            self.get_logger().info(f"Got scehdule: %s" % json.dumps(self.schedule))

    def process_raw_schedule(self, raw):
        schedule = {}
        # expand raw to the scene schedule
        for scene in raw:
            if (scene != 'type') and (scene!= 'filepath'):
                # make the day array
                if raw[scene].has_key('day'):
                    raw[scene]['day'] = self.expand_day_str(raw[scene]['day'])
                else:
                    raw[scene]['day'] = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
                if raw[scene].has_key('time'):
                   raw[scene]['time_sec'] = self.convert_time_str(raw[scene]['time'])


    def expand_day_str(self,day_cfg_str):
        day_list = []
        for day_str in day_cfg_str:
            if day_str.title()=='All' or day_str.title()=='Everyday':
                day_list.extend(["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"])
            elif day_str.title()=='Weekdays' or day_str.title()=='Workdays':
                day_list.extend(["Monday", "Tuesday", "Wednesday", "Thursday","Friday"])
            elif day_str.title()=='Weekend'or day_str.title()=='Weekends':
                day_list.extend(["Saturday", "Sunday"])
            else:
                day_list.extend(day_str)    
        return day_list

    def today_str(self):
        now = datetime.datetime.now()
        self.today = now.strftime("%A")
        return self.today

    def sun_listener_callback(self,msg):
        msg = json.loads(msg.data)
        self.named_events = msg['payload']['events']
        self.need_named_events = False        

    def convert_time_str(self,timestr):
        pluses = timestr.count('+')
        minuses = timestr.count('-')
        if minuses + pluses > 1:
            self.get_logger().error(f"Can't parse time: %s (expression too complex)" % timestr)
        if pluses > 1:
            self.get_logger().error(f"Can't parse time: %s (too many pluses)" % timestr)
        if pluses == 1:
            split = timestr.split('+',2)
            namedevent = split[0]
            tstr = split[1]
            mul = 1.0
        elif minuses == 1:
            split = timestr.split('-',2)
            namedevent = split[0]
            tstr = split[1]
            mul = -1.0
        else:
            namedevent=''
            tstr = timestr
            mul = 1.0
        colons = tstr.count(':')
        if colons == 2:
            st = time.strptime(tstr,'%H:%M:%S')
            seconds = datetime.timedelta(hours=st.tm_hour,minutes=st.tm_min,seconds=st.tm_sec).total_seconds()
        elif colons == 1:
            st = time.strptime(tstr,'%H:%M')
            seconds = datetime.timedelta(hours=st.tm_hour,minutes=st.tm_min).total_seconds()
        if self.need_named_events == True and len(namedevent)> 0:
            self.get_logger().warn(f"Can't find named event: %s (no named events received, yet)" % namedevent)
            return nan
        
        # get to here seconds is either a time since midnight, or a time modifier on the named event time

def main(args=None):
    rclpy.init(args=args)

    home_scheduler = HomeScheduler()

    rclpy.spin(home_scheduler)

    home_scheduler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

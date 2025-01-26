# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
from cmath import nan
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import date, time, datetime, timedelta
import json
from dateutil import parser
import math
import pytz

class HomeScheduler(Node):
    # The home_scheduler node takes configuration from the "settings" topic.
    # it relies on the config node to publish data from the file "schedule.ini"
    # and uses the schedule to publish a message on the "scenes" topic, every
    # so often.
    def __init__(self):
        super().__init__('home_scheduler')
        self.publisher_ = self.create_publisher(String, '/lighting/scenes', 10)
        self.timer_period = 3.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.config_subscription = self.create_subscription(
            String,
            '/home/configuration',
            self.config_listener_callback,
            10)
        self.config_subscription
        self.sun_subscription = self.create_subscription(
            String,
            '/environment/sun',
            self.sun_listener_callback,
            10)
        self.config_subscription
        self.i = 0
        self.need_config_schedule = True
        self.need_named_events = True
        self.schedule = {}
        self.raw_schedule = {}
        self.today_schedule = {}


    def timer_callback(self):
        msg = String()
        m={}
        m['index'] = self.i
        m['interval'] = self.timer_period
        m['payload'] = {}
        m['payload']['today'] = self.today_str()
        self.process_todays_schedule()
        m['payload']['next_scene'] = self.next_scene
        m['payload']['previous_scene'] = self.prev_scene
        m['payload']['secs_remaining'] = self.secs_remaining
        m['payload']['secs_elapsed'] = self.secs_elapsed
        m['payload']['/lighting/scenes'] = self.today_schedule
        msg.data = json.dumps(m)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing schedule with %d scenes' % len(m['payload']['/lighting/scenes']))
        self.i += 1
        self.need_config_location = True


    def config_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "SCHEDULE":
            self.raw_schedule = msg['payload']
            self.get_logger().info(f"Got scehdule: %s" % json.dumps(self.raw_schedule))
            self.schedule = self.process_raw_schedule(self.raw_schedule)
            self.need_config_schedule = False
        if msg['payload']['type'] == "LOCATION":
            self.latitude = float(msg['payload']['Coordinates']['latitude']) 
            self.longitude = float(msg['payload']['Coordinates']['longitude']) 
            self.city = msg['payload']['City']['city']              
            self.state = msg['payload']['City']['state']
            self.tzname = msg['payload']['Timezone']['tzname']
            self.need_config_location = False
            self.get_logger().info(f"Got config: Location is {self.city}, {self.state}")
   
    def process_raw_schedule(self, raw):
        schedule = {}
        # expand raw to the scene schedule
        for scene in raw:
            if (scene != 'type') and (scene!= 'filepath'):
                # make the day array
                if 'day' in raw[scene]:
                    raw[scene]['day'] = self.parse_day_str(raw[scene]['day'])
                    # self.get_logger().info(f"Scene '%s' has 'day' '%s'" % (scene,raw[scene]['day']))
                else:
                    raw[scene]['day'] = ["Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"]
                    self.get_logger().warn(f"Scene '%s' has no entry 'day' (will run every day)" % scene)
                # compute the seconds since midnight    
                if 'time' in raw[scene]: 
                   raw[scene]['time_seconds'] = self.parse_time_str(raw[scene]['time'])
                   # self.get_logger().info(f"Scene '%s' has 'time' '%.1f'" % (scene,raw[scene]['time_seconds']))
                else:
                   raw[scene]['time_seconds'] = nan  
                    
                if raw[scene]['time_seconds'] != nan: 
                    # print(f"adding scene '%s' to schedule" % scene )      
                    schedule[scene] = raw[scene]
                else:
                    self.get_logger().error(f"Scene '%s' time not parsed" % scene)
        if len(schedule): 
            # self.show_schedule(schedule)   
            self.get_logger().info(f"Scene schedule: %s" % json.dumps(schedule))
        else:
            self.get_logger().warn(f"No scene schedule, yet.")    
        return schedule

    def parse_day_str(self,day_cfg_str_or_list):
        day_list = []
        if isinstance(day_cfg_str_or_list,list):  # is it a list or just a value (string?)
            for day_str in day_cfg_str_or_list:
                    day_list = self.append_special_string(day_list, day_str)
        else:
            day_list = self.append_special_string(day_list,day_cfg_str_or_list)
        return day_list

    def append_special_string(self,in_list,day_str):
        day_list = in_list
        if day_str.title()=='All' or day_str.title()=='Everyday':
            day_list = ['Monday', 'Tuesday', 'Wednesday', 'Thursday', 'Friday', 'Saturday', 'Sunday']
        elif day_str.title()=='Weekdays' or day_str.title()=='Workdays':
            day_list.append("Monday")
            day_list.append("Tuesday")
            day_list.append("Wednesday")
            day_list.append("Thursday")
            day_list.append("Friday")
        elif day_str.title()=='Weekend'or day_str.title()=='Weekends':
            day_list.append("Saturday")
            day_list.append("Sunday")
        else:
            day_list.append(day_str)    
        return day_list    
   
    def parse_time_str(self,timestr):
        pluses = timestr.count('+')
        minuses = timestr.count('-')
        if minuses + pluses > 1:
            self.get_logger().error(f"Can't parse time: '%s' (expression too complex)" % timestr)
        if pluses > 1:
            self.get_logger().error(f"Can't parse time: '%s' (too many pluses)" % timestr)
        if minuses > 1:
            self.get_logger().error(f"Can't parse time: '%s' (too many minuses)" % timestr)
        if pluses == 1:
            split = timestr.split('+',2)
            namedevent = split[0].strip()
            tstr = split[1].strip()
            mul = 1.0
        elif minuses == 1:
            split = timestr.split('-',2)
            namedevent = split[0].strip()
            tstr = split[1].strip()
            mul = -1.0
        else:
            namedevent=''
            tstr = timestr
            mul = 1.0
        colons = tstr.count(':')
        if colons == 2:
            dt = parser.parse(tstr)
            seconds = timedelta(hours=dt.hour,minutes=dt.minute,seconds=dt.second).total_seconds()
        elif colons == 1:
            dt = parser.parse(tstr)
            seconds = timedelta(hours=dt.hour,minutes=dt.minute).total_seconds()
        else:
            namedevent=tstr 
        if len(namedevent) > 0:    
            if self.need_named_events == True:
                self.get_logger().warn(f"Can't find named event: '%s' (no named events received, yet)" % namedevent)
                return nan
            # get to here seconds is either a time since midnight, or a time modifier on the named event time
            else:
                if namedevent in self.named_events:
                    named_event_seconds = self.named_events[namedevent]
                else:
                    self.get_logger().warn(f"Can't find named event: '%s' (not in list)" % namedevent)    
                    return nan
            return named_event_seconds + mul * seconds 
        else:
            return seconds
 
    def today_str(self):
        now = datetime.now()
        self.today = now.strftime("%A")
        return self.today

    def sun_listener_callback(self,msg):
        msg = json.loads(msg.data)
        self.get_logger().info(f"Got sun events: %s" % json.dumps(msg['payload']['/home/events']))
        self.named_events = msg['payload']['events']
        if len(self.raw_schedule): 
            self.process_raw_schedule(self.raw_schedule) # reprocess schedule with latest sun events
        self.need_named_events = False        
       
    def show_schedule(self,schedule):
        print("--------------------------")
        for scene in schedule:
            print(f"%s:" % scene)
            for field in schedule[scene]:
                print(f"\t%s:" % field)
                if isinstance(schedule[scene][field],list):
                    for list_or_string in schedule[scene][field]:
                        print(f"\t\t%s" % list_or_string)    
                else:    
                    print(f"\t\t%s" % schedule[scene][field])      
        print("--------------------------")            
    
    def process_todays_schedule(self):
        self.today_schedule = {}    
        if len(self.schedule) > 0 :
            for scene in self.schedule:
                scene_dict = self.schedule[scene]   
                day_dict = scene_dict['day']           
                if self.today in day_dict:
                    if not math.isnan(scene_dict['time_seconds']):
                        self.today_schedule[scene] = scene_dict['time_seconds'] 
        if self.need_config_location:
            now = datetime.now()
            midnight = datetime.combine(datetime.today(), time.min)
        else:
            home_tz = pytz.timezone(self.tzname)
            now = home_tz.localize(datetime.now())
            midnight = datetime.combine(datetime.today(), time.min)
            midnight = home_tz.localize(midnight)         
        adjust_to_now = (now - midnight).total_seconds()
        min_sec = 24 * 3600
        min_scene = 'Unknown'
        max_sec = -24 * 3600
        max_scene = 'Unknown'
        for key in self.today_schedule:
            value = self.today_schedule[key]
            value = value - adjust_to_now
            if (value >= 0.0) and (value < min_sec): 
                min_sec = value
                min_scene = key
            if (value <= 0.0) and (value > max_sec):
                max_sec = value
                max_scene = key
        self.next_scene = min_scene
        self.secs_remaining = min_sec
        self.prev_scene = max_scene
        self.secs_elapsed = abs(max_sec)

                    
def main(args=None):
    rclpy.init(args=args)

    home_scheduler = HomeScheduler()

    rclpy.spin(home_scheduler)

    home_scheduler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

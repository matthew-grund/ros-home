# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import date, time, datetime
import pytz
from astral import LocationInfo
from astral.sun import sun
from astral.sun import golden_hour
from astral.sun import SunDirection

class SunTracker(Node):

    def __init__(self):
        super().__init__('sun_tracker')
        self.publisher_ = self.create_publisher(String, '/environment/sun', 10)
        self.timer_period = 10.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.config_subscription = self.create_subscription(
            String,
            '/home/configuration',
            self.config_listener_callback,
            10)
        self.config_subscription
        self.i = 0
        self.sun_data = {}
        self.need_config_location = True

    def timer_callback(self):
        if self.need_config_location:
            return  # don't do anything until we get location
        self.update_sun_track()
        sundict = {}
        sundict['next_event'] = self.next_event
        sundict['previous_event'] = self.prev_event
        sundict['secs_remaining'] = self.secs_remaining
        sundict['secs_elapsed'] = self.secs_elapsed
        sundict['events'] = self.sun_event
        msgdict = {}
        msgdict['index'] = self.i
        msgdict['interval'] = self.timer_period
        msgdict['payload'] = sundict
        msgjson = json.dumps(msgdict)
        msg = String()
        msg.data = msgjson
        self.publisher_.publish(msg)
        self.get_logger().info('Sun: "%s"' % msg.data)
        self.i += 1

    def update_sun_track(self):
        
        if self.need_config_location:
            now = datetime.now()
            midnight = datetime.combine(datetime.today(), time.min)
        else:
            home_tz = pytz.timezone(self.tzname)
            now = home_tz.localize(datetime.now())
            midnight = datetime.combine(datetime.today(), time.min)
            midnight = home_tz.localize(midnight)
        home_sun = sun(self.location.observer)
        gham = golden_hour(self.location.observer, midnight.date(), SunDirection.RISING)   # use midnight to keep on the current day
        ghpm = golden_hour(self.location.observer, midnight.date(), SunDirection.SETTING)

        # seconds since midnight for today's sun events
        self.sun_event={}
        self.sun_event['Dawn'] = (home_sun['dawn'] - midnight).total_seconds()
        self.sun_event['Sunrise'] = (home_sun['sunrise'] - midnight).total_seconds()
        self.sun_event['GoldenHourEnd'] = (gham[1] - midnight).total_seconds()         
        self.sun_event['SolarNoon'] = (home_sun['noon'] - midnight).total_seconds()
        self.sun_event['GoldenHourStart'] = (ghpm[0] - midnight).total_seconds()       
        self.sun_event['Sunset'] = (home_sun['sunset'] - midnight).total_seconds()     
        self.sun_event['Dusk'] = (home_sun['dusk'] - midnight).total_seconds()

        adjust_to_now = (now - midnight).total_seconds()
        min_sec = 24 * 3600
        min_event = 'Unknown'
        max_sec = -24 * 3600
        max_event = 'Unknown'
        for key in self.sun_event:
            value = self.sun_event[key]
            value = value - adjust_to_now
            if (value >= 0.0) and (value < min_sec): 
                min_sec = value
                min_event = key
            if (value <= 0.0) and (value > max_sec):
                max_sec = value
                max_event = key
        self.next_event = min_event
        self.secs_remaining = min_sec
        self.prev_event = max_event
        self.secs_elapsed = abs(max_sec)


    def config_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "LOCATION":
            self.latitude = float(msg['payload']['Coordinates']['latitude']) 
            self.longitude = float(msg['payload']['Coordinates']['longitude']) 
            self.city = msg['payload']['City']['city']              
            self.state = msg['payload']['City']['state']
            self.tzname = msg['payload']['Timezone']['tzname']
            self.location = LocationInfo(self.city, self.state, self.tzname, self.latitude, self.longitude)
            self.need_config_location = False
            self.get_logger().info(f"Got config: Location is {self.city}, {self.state}")


def main(args=None):
    rclpy.init(args=args)

    sun_tracker = SunTracker()

    rclpy.spin(sun_tracker)

    sun_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

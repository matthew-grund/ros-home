# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
import pytz
from astral import LocationInfo
from astral.sun import sun
from astral.sun import golden_hour
from astral.sun import SunDirection

class SunTracker(Node):

    def __init__(self):
        super().__init__('sun_tracker')
        self.publisher_ = self.create_publisher(String, 'sun', 10)
        self.timer_period = 10.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.city = "Mashpee"
        self.state = "Massachusetts"
        self.tzname = "America/New_York"
        self.latitude = 41.619
        self.longitude = -70.451
        self.location = LocationInfo(self.city, self.state, self.tzname, self.latitude, self.longitude)
        self.sun_data = {}

    def timer_callback(self):
        self.update_sun_track()
        sundict = {}
        sundict['next_event'] = self.next_event
        sundict['prev_event'] = self.prev_event
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
        home_tz = pytz.timezone(self.tzname)
        now = home_tz.localize(datetime.now())
        home_sun = sun(self.location.observer)
        gham = golden_hour(self.location.observer, datetime.now().date(), SunDirection.RISING)
        ghpm = golden_hour(self.location.observer, datetime.now().date(), SunDirection.SETTING)

        self.sun_event={}
        self.sun_event['Dawn'] = (home_sun['dawn'] - now).total_seconds()
        self.sun_event['Sunrise'] = (home_sun['sunrise'] - now).total_seconds()
        self.sun_event['GoldenHourEnd'] = (gham[1] - now).total_seconds()
        self.sun_event['SolarNoon'] = (home_sun['noon'] - now).total_seconds()
        self.sun_event['GoldenHourStart'] = (ghpm[0] - now).total_seconds()
        self.sun_event['Sunset'] = (home_sun['sunset'] - now).total_seconds()
        self.sun_event['Dusk'] = (home_sun['dusk'] - now).total_seconds()

        min_sec = 24 * 3600
        min_event = 'Unknown'
        max_sec = -24 * 3600
        max_event = 'Unknown'
        for key in self.sun_event:
            value = self.sun_event[key]
            if (value >= 0.0) and (value < min_sec):
                min_sec = value
                min_event = key
            if (value <= 0.0) and (value > max_sec):
                max_sec = value
                max_event = key
        self.next_event = min_event
        self.secs_remaining = min_sec
        self.prev_event = max_event
        self.secs_elapsed = max_sec


def main(args=None):
    rclpy.init(args=args)

    sun_tracker = SunTracker()

    rclpy.spin(sun_tracker)

    sun_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

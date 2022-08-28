# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

class WeatherTracker(Node):

    def __init__(self):
        super().__init__('weather_tracker')
        self.publisher_ = self.create_publisher(String, 'weather', 10)
        self.timer_period = 30.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.city = "Mashpee"
        self.state = "Massachusetts"
        self.tzname = "America/New_York"
        self.latitude = 41.619
        self.longitude = -70.451
        self.need_forecast_url = True

    def timer_callback(self):
        if self.need_forecast_url:
            '''https://api.weather.gov/points/41.619,-70.451'''
            get_url =  "https://api.weather.gov/points/%.4f,%.4f" % (self.latitude,self.longitude)
            try:    
                geo_result = requests.get(get_url)
            except:
                geo_result.status_code = 404
                geo_result.text = ""
            if geo_result.status_code == 200:
                geo = json.loads(geo_result.text)
                self.forecast_url = geo['properties']['forecast']
                self.need_forecast_url = False
                self.get_logger().info('Forecast URL: "%s"' % self.forecast_url)
        else:
            try:    
                result = requests.get(self.forecast_url)
            except:
                result.status_code = 404
                result.text = ""
            if result.status_code == 200:
                res = json.loads(result.text)
                forecast = res['properties']['periods']
                m = {}
                m['index'] = self.i
                m['interval'] = self.timer_period
                m['payload'] = forecast
                msg = String()
                msg.data = json.dumps(m)
                self.publisher_.publish(msg)
                self.get_logger().info('Weather: "%s"' % forecast[0]['detailedForecast'])
                self.i += 1

def main(args=None):
    rclpy.init(args=args)

    weather_tracker = WeatherTracker()

    rclpy.spin(weather_tracker)

    weather_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

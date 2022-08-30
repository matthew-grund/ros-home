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
        self.publisher_wx = self.create_publisher(String, 'wx_forecast', 10)
        self.publisher_conditions = self.create_publisher(String, 'wx_conditions', 10)   
        self.publisher_alerts = self.create_publisher(String, 'wx_alerts', 10)      
        self.timer_period = 30.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.city = "Mashpee"             # FIXME: add to an INI file, or parameter server 
        self.state = "Massachusetts"
        self.tzname = "America/New_York"
        self.latitude = 41.619
        self.longitude = -70.451
        self.need_forecast_url = True
        self.need_obs_station_url = True

    def timer_callback(self):
        if self.need_forecast_url:
            '''https://api.weather.gov/points/41.619,-70.451'''
            get_url =  "https://api.weather.gov/points/%.4f,%.4f" % (self.latitude,self.longitude)
            try:    
                geo_result = requests.get(get_url)
            except:
                return
            if geo_result.status_code == 200:
                geo = json.loads(geo_result.text)
                self.forecast_url = geo['properties']['forecast']
                self.stations_url = geo['properties']['observationStations']
                self.need_forecast_url = False
                self.get_logger().info('Forecast URL: "%s"' % self.forecast_url)

        elif self.need_obs_station_url:
            try:    
                stations_result = requests.get(self.stations_url)
            except:
                return
            if stations_result.status_code == 200:
                stations = json.loads(stations_result.text)
                self.observation_url = stations['features'][0]['id'] +"/observations"  # assumes first station in list is closest
                self.obs_station_name = stations['features'][0]['properties']['name']    
                self.need_obs_station_url = False
                self.get_logger().info('Observation Station: "%s"' % self.observation_url)
        
        else:
            # current conditions
            try:    
                result = requests.get(self.observation_url)
            except:
                return
            if result.status_code == 200:
                res = json.loads(result.text)
                observations = res['features'][0]['properties']['periods'][0]   # just the current observation
                m = {}
                m['index'] = self.i
                m['interval'] = self.timer_period
                m['payload'] = observations
                msg = String()
                msg.data = json.dumps(m)
                self.publisher_conditions.publish(msg)
                self.get_logger().info('Current: "%s" %.1f F' % (observations['textDescription'],observations['temperature']['value']/5*9+32))
                self.i += 1

            # forecast
            try:    
                result = requests.get(self.forecast_url)
            except:
                return
            if result.status_code == 200:
                res = json.loads(result.text)
                forecast = res['properties']['periods']
                m = {}
                m['index'] = self.i
                m['interval'] = self.timer_period
                m['payload'] = forecast
                msg = String()
                msg.data = json.dumps(m)
                self.publisher_wx.publish(msg)
                self.get_logger().info('Forecast: "%s"' % forecast[0]['detailedForecast'])
                self.get_logger().info('Forecast: "%s"' % forecast[1]['detailedForecast'])
                self.get_logger().info('Forecast: "%s"' % forecast[2]['detailedForecast'])
                self.i += 1

def main(args=None):
    rclpy.init(args=args)

    weather_tracker = WeatherTracker()

    rclpy.spin(weather_tracker)

    weather_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

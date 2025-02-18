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
    # the weather tracker node provides forecasts and current conditions, by interacting 
    # with the U.S. N.O.A.A. weather service.
    #
    def __init__(self):
        super().__init__('weather_tracker')
        self.publisher_wx = self.create_publisher(String, '/environment/weather/forecast', 10)
        self.publisher_conditions = self.create_publisher(String, '/environment/weather/conditions', 10)   
        self.publisher_alerts = self.create_publisher(String, '/environment/weather/alerts', 10)  
        self.publisher_commands = self.create_publisher(String,'/home/commands/raw', 10)    
        self.timer_period = 30.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.config_subscription = self.create_subscription(
            String,
            '/home/configuration',
            self.config_listener_callback,
            10)
        self.config_subscription
        self.i = 0
        self.ticks_elapsed = 0
        self.need_forecast_url = True
        self.need_observation_stations = True
        self.need_config_location = True
        self.get_logger().info(f"Waiting for config message on topic '/home/configuration'")
        

    def timer_callback(self):
        if self.need_config_location:
            self.ticks_elapsed += 1
            if self.ticks_elapsed >2:
                msg = String()
                msg.data = 'configure location'  # ask the configuration thing to send location
                self.publisher_commands.publish(msg)
                self.get_logger().warning(f"No config after {self.timer_period * self.ticks_elapsed} sec. sending command.")
            return

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
                self.get_logger().info('Observation Stations URL: "%s"' % self.stations_url)

        elif self.need_observation_stations:
            try:    
                stations_result = requests.get(self.stations_url)
            except:
                return
            if stations_result.status_code == 200:
                stations = json.loads(stations_result.text)
                # self.get_logger().info(f"available observation stations: '{stations}'")
                feature_count = 0
                self.stations = {}
                for feature in stations['features']:
                    feature_count += 1
                    if feature_count < 10: # only grab the closest few stations
                        station_id = feature['properties']['stationIdentifier']
                        station = {}
                        station['id'] = station_id
                        station['name'] = feature['properties']['name']
                        station['location'] = feature['geometry']['coordinates']
                        station['url'] = feature['id'] +"/observations"  
                        self.stations[station_id] = station
                        self.get_logger().info(f"found observation station '{station_id}': {station['name']}")
                self.need_observation_stations = False
                self.get_logger().info(f"Found {feature_count} observation stations, using {len(self.stations)}")
                self.station_list=[]
                for id in self.stations:
                    self.station_list.append(id)
                self.next_station_index = 0    
        else:
            # current conditions at next one of known stations
            station = self.stations[self.station_list[self.next_station_index]]
            self.next_station_index += 1
            if self.next_station_index >= len(self.station_list):
                self.next_station_index = 0
            try:    
                result = requests.get(station['url'])
            except:
                return
            if result.status_code == 200:
                res = json.loads(result.text)
                observations = res['features'][0]['properties']   # just the current observation
                if (type(observations['temperature']['value']) == int) or \
                    (type(observations['temperature']['value']) == float):
                    obs_has_temp = True
                    temp_f = observations['temperature']['value']/5*9+32
                else:
                    obs_has_temp = False
                if (type(observations['windSpeed']['value']) == int) or \
                    (type(observations['windSpeed']['value']) == float):
                    obs_has_windspeed = True
                    windspeed_kts = observations['windSpeed']['value'] * 0.539957
                else:
                    obs_has_windspeed = False 
                if (type(observations['windDirection']['value']) == int) or \
                    (type(observations['windDirection']['value']) == float):
                    obs_has_winddir = True
                    winddir_deg = observations['windDirection']['value']
                else:
                    obs_has_winddir = False    

                if obs_has_windspeed and obs_has_winddir and obs_has_temp:
                    summary=f"Conditions at {station['name']} [{station['id']}] - {observations['textDescription']} {int(temp_f)}{u"\u00B0"}F {int(windspeed_kts)}kts {self.deg_to_dir_str(winddir_deg)}"
                elif obs_has_temp:
                    summary=f"Conditions at {station['name']} [{station['id']}] - {observations['textDescription']} {int(temp_f)}{u"\u00B0"}F (NULL WIND)"
                elif obs_has_windspeed and obs_has_winddir:
                    summary=f"Conditions at {station['name']} [{station['id']}] - {observations['textDescription']} {int(windspeed_kts)}kts {self.deg_to_dir_str(winddir_deg)} (NULL TEMP)"
                else:
                    summary=f"Conditions at {station['name']} [{station['id']}] - {observations['textDescription']} (NULL TEMP, NULL WIND)"
                self.get_logger().info(summary)    
                m = {}
                m['index'] = self.i
                m['interval'] = self.timer_period
                m['payload'] = {'id':station['id'],'summary':summary,'observations':observations}
                msg = String()
                msg.data = json.dumps(m)
                self.publisher_conditions.publish(msg)
            else:
                self.get_logger().warning(f"Bad response {result.status_code} from {station['url']}")    
            self.i += 1
            if self.i > 30: # every 15 minutes fetch the base URLs again
                self.i = 0
                self.need_forecast_url = True
                self.need_observation_stations = True
                    
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
                for f1 in forecast:
                    self.get_logger().info(f"Forecast for {f1['name']} - {f1['detailedForecast']}")
                self.i += 1

                                           
    def config_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "LOCATION":
            self.latitude = float(msg['payload']['Coordinates']['latitude']) 
            self.longitude = float(msg['payload']['Coordinates']['longitude']) 
            self.city = msg['payload']['City']['city']              
            self.state = msg['payload']['City']['state']
            self.tzname = msg['payload']['Timezone']['tzname']
            self.need_config_location = False
            self.get_logger().info(f"Got config: Location is {self.city}, {self.state}")


    def deg_to_dir_str(self,deg:float):
        if deg >= 345 or deg <= 15:
            return 'N'
        if deg > 15 and deg < 35:
            return 'NNE'
        if deg >= 35 and deg <= 55 :
            return 'NE'
        if deg > 55 and deg < 75 :
            return 'ENE'
        if deg >= 75  and deg <= 105 :
            return 'E'
        if deg > 105 and deg < 125:
            return 'ESE'
        if deg >= 125  and deg <= 145 :
            return 'SE'
        if deg > 145 and deg < 165 :
            return 'SSE'
        if deg >= 165 and deg <= 195 :
            return 'S'
        if deg > 195 and deg < 215 :
            return 'SSW'
        if deg >= 215 and deg <= 235 :
            return 'SW'
        if deg > 235 and deg < 255 :
            return 'WSW'
        if deg >= 255 and deg <= 285 :
            return 'W'
        if deg > 285 and deg < 305 :
            return 'WNW'
        if deg >= 305 and deg <= 325:
            return 'NW'
        if deg > 325 and deg < 345 :
            return 'NNW'
        return str(deg) + '???'


def main(args=None):
    rclpy.init(args=args)

    weather_tracker = WeatherTracker()

    rclpy.spin(weather_tracker)

    weather_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

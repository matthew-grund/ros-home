# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#

from cmath import nan
#from distutils.log import set_verbosity
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import datetime
import re

class EventDetector(Node):
    """
    EventDetector is a ROS node that subscribes to various topics to detect and handle events related to devices, people tracking, lighting, weather, and more. It publishes events and known devices to specific topics.
    Attributes:
        lighting_status (dict): Dictionary to store the status of lights.
        new_devices (list): List to store newly discovered devices.
        known_devices (list): List to store known devices.
        known_nodes (dict): Dictionary to store known nodes.
        forecasts (list): List to store weather forecasts.
        subscription_devices (Subscription): Subscription to the '/devices/discovered/network' topic.
        subscription_owntracks (Subscription): Subscription to the '/people/tracking/owntracks' topic.
        subscription_scene (Subscription): Subscription to the '/lighting/scenes' topic.
        subscription_sun (Subscription): Subscription to the '/environment/sun' topic.
        subscription_forecast (Subscription): Subscription to the '/environment/weather/forecast' topic.
        subscription_conditions (Subscription): Subscription to the '/environment/weather/conditions' topic.
        subscription_json_commands (Subscription): Subscription to the '/home/commands/json' topic.
        subscription_raw_commands (Subscription): Subscription to the '/home/commands/raw' topic.
        subscription_wx_alerts (Subscription): Subscription to the '/environment/weather/alerts' topic.
        subscription_node_list (Subscription): Subscription to the '/nodes/list' topic.
        subscription_lighting_status (Subscription): Subscription to the '/lighting/status' topic.
        publisher_events (Publisher): Publisher for the '/home/events' topic.
        period_name (str): Name of the current period.
        timer_period (float): Timer period in seconds.
        timer (Timer): Timer to trigger periodic callbacks.
        i (int): Counter for the timer callback.
        dev_index (int): Index for device events.
        event_index (int): Index for events.
        publisher_devices (Publisher): Publisher for the '/devices/known/network' topic.
        wx_latest_conditions (dict): Dictionary to store the latest weather conditions.
        wx_recent_forecasts (dict): Dictionary to store recent weather forecasts.
        lights (list): List to store light information.
        num_lights_on (int): Number of lights that are currently on.
    Methods:
        __init__(): Initializes the EventDetector node and sets up subscriptions and publishers.
        timer_callback(): Callback function for the timer to periodically prune the forecast list.
        prune_forecast_list(): Prunes the forecast list by removing expired forecasts.
        json_commands_callback(msg): Callback function to handle incoming JSON commands.
        raw_commands_callback(msg): Callback function to handle incoming raw commands.
        parse_command(argv): Parses and processes incoming commands.
        publish_event(typename, severity, desc, detail): Publishes an event with the given details.
        publish_known_devices(): Publishes the list of known devices.
        devices_callback(msg): Callback function to handle incoming device discovery messages.
        owntracks_callback(msg): Callback function to handle incoming OwnTracks messages.
        sun_callback(msg): Callback function to handle incoming sun status messages.
        scene_callback(msg): Callback function to handle incoming scene messages.
        forecast_callback(msg): Callback function to handle incoming weather forecast messages.
        add_forecast(forecast): Adds a new forecast to the forecast list.
        publish_forecast_event(forecast): Publishes a forecast event.
        publish_forecast_summary_event(): Publishes a summary of the forecast events.
        conditions_callback(msg): Callback function to handle incoming weather conditions messages.
        wx_alerts_callback(msg): Callback function to handle incoming weather alerts messages.
        node_list_callback(msg): Callback function to handle incoming node list messages.
        lighting_status_callback(msg): Callback function to handle incoming lighting status messages.
        detect_device_event(): Detects and handles device events based on the current and known devices.
    """
    

    # Initialize the EventDetector node
    def __init__(self):
        super().__init__('home_event_detect')
        self.lighting_status={}
        self.new_devices=[]
        self.known_devices=[]
        self.known_nodes={}
        self.forecasts=[]
        self.sun_summary = ""
        self.subscription_devices = self.create_subscription(
            String,
            '/devices/discovered/network',
            self.devices_callback,
            10)
        self.subscription_devices  # prevent unused variable warning

        self.subscription_owntracks = self.create_subscription(
            String,
            '/people/tracking/owntracks',
            self.owntracks_callback,
            10)
        self.subscription_owntracks  # prevent unused variable warning

        self.subscription_scene = self.create_subscription(
            String,
            '/lighting/scenes',
            self.scene_callback,
            10)
        self.subscription_scene  # prevent unused variable warning

        self.subscription_sun = self.create_subscription(
            String,
            '/environment/sun',
            self.sun_callback,
            10)
        self.subscription_sun  # prevent unused variable warning

        self.subscription_forecast = self.create_subscription(
            String,
            '/environment/weather/forecast',
            self.forecast_callback,
            10)
        self.subscription_forecast  # prevent unused variable warning

        self.subscription_conditions = self.create_subscription(
            String,
            '/environment/weather/conditions',
            self.conditions_callback,
            10)
        self.subscription_conditions  # prevent unused variable warning

        self.subscription_json_commands = self.create_subscription(
            String,
            '/home/commands/json',
            self.json_commands_callback,
            10)

        self.subscription_raw_commands = self.create_subscription(
            String,
            '/home/commands/raw',
            self.raw_commands_callback,
            10)

        self.subscription_wx_alerts = self.create_subscription(
            String,
            '/environment/weather/alerts',
            self.wx_alerts_callback,
            10)
        self.subscription_wx_alerts # prevent unused variable warning

        self.subscription_node_list = self.create_subscription(
            String,
            '/nodes/list',
            self.node_list_callback,
            10)
        self.subscription_node_list # prevent unused variable warning

        self.subscription_lighting_status = self.create_subscription(
            String,
            '/lighting/status',
            self.lighting_status_callback,
            10)
        self.subscription_lighting_status # prevent unused variable warning

        self.publisher_events = self.create_publisher(String, '/home/events', 10)
        self.period_name = ""
        self.timer_period = 10.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.dev_index = 0
        self.event_index = 0
        self.publisher_devices = self.create_publisher(String, '/devices/known/network', 10)
        self.wx_latest_conditions = {}
        self.wx_recent_forecasts = {}
        self.lights = []
        self.num_lights_on = -1


    def timer_callback(self):
        """
        Callback function for the timer to periodically prune the forecast list.
        """
        self.i += 1
        self.prune_forecast_list()


    def prune_forecast_list(self):
        """
        Prunes the forecast list by removing expired forecasts.
        """
        now = datetime.datetime.now().isoformat()
        f_len = len(self.forecasts)
        if f_len < 2:
            return
        new_forecasts = []
        pruned_names = []
        for f_tuple in self.forecasts:
            expiry_timestamp = f_tuple[0]
            if expiry_timestamp > now:
                new_forecasts.append(f_tuple)
            else:
                pruned_names.append(f_tuple[1]["name"])
        if len(pruned_names) > 0:
            self.get_logger().info(f"Pruned forecast for '{"', ".join(pruned_names)}'")
            self.publish_forecast_event(new_forecasts[0][1])
        self.forecasts = new_forecasts


    def json_commands_callback(self,msg):
        """
        Callback function to handle incoming JSON commands.
        Parses the JSON message and processes the command payload.
        """
        m = json.loads(msg.data)
        # self.device_interval = m['interval']
        command = m['payload']
        #if m['index'] == 0:
        #    self.publish_event('DEV','ERROR','Device discovery node restarted',self.new_devices)
        if "argv" in command:
            argv = command['argv']
            self.parse_command(argv)


    def raw_commands_callback(self,msg):
        """
        Callback function to handle incoming raw commands.
        Parses the raw message and processes the command payload.
        """
        command = msg.data
        argv=command.split()
        self.parse_command(argv)


    def parse_command(self,argv):
        """
        Parses and processes incoming commands.
        """
        cmd_str = " ".join(argv)
        self.get_logger().info(f"Got command: '{cmd_str}'")
        # FIXME: this is not a very functional command shell
        if "EVENT" in argv[0].upper():
            # if len(argv) == 1:
            self.get_logger().warning(f"Got a command, doing nothing")
            self.files = {}
        else:
            self.get_logger().info(f"Command not for event_detect")


    def publish_event(self, typename, severity, desc, detail):
        """
        Publishes an event with the given details.
        """
        msg = String()
        event = {}
        event['event_type'] = typename
        event['severity'] = severity
        event['timestamp'] = datetime.datetime.now().isoformat()
        event['description'] = desc
        event['detail'] = detail
        m = {}
        m['index'] = self.event_index
        m['interval'] = nan
        m['payload'] = event
        msg.data = json.dumps(m)
        self.publisher_events.publish(msg)
        if severity=='ERROR':
            self.get_logger().error(desc)
        else:
            self.get_logger().warning(desc)
        self.event_index += 1    


    def publish_known_devices(self):
        """
        Publishes the list of known devices.
        """     
        msg = String()
        msgdict = {}
        msgdict['index'] = self.i
        msgdict['interval'] = self.device_interval
        msgdict['payload'] = self.known_devices
        msg.data = json.dumps(msgdict)
        self.publisher_devices.publish(msg)
        self.get_logger().info('Devices: Publishing known devices list: %d devices' % len(self.known_devices))


    def devices_callback(self, msg):
        """
        Callback function to handle incoming device discovery messages.
        """
        m = json.loads(msg.data)
        self.device_interval = m['interval']
        self.new_devices = m['payload']
        #if m['index'] == 0:
        #    self.publish_event('DEV','ERROR','Device discovery node restarted',self.new_devices)
        self.detect_device_event()


    def owntracks_callback(self, msg):
        """
        Callback function to handle incoming OwnTracks messages.
        """
        self.get_logger().info('Owntracks: "%s"' % msg.data)
        md = json.loads(msg.data)
        track = md['payload']
        if track['type'] == 'EVENT':
            self.publish_event('TRACK','INFO',track['description'],track)


    def sun_callback(self, msg):
        """
        Callback function to handle incoming sun status messages.
        """ 
        m = json.loads(msg.data)
        max_sec = m['interval']
        sun = m['payload']
        #if m['index'] == 0:
        #    self.publish_event('/environment/sun','ERROR','Sun tracker node restarted',sun)
        rem_sec = sun['secs_remaining']
        rem_hours = rem_sec / 3600.0
        event1 = sun['next_event']
        # convert from camel case to spaces
        event = re.sub(r'([a-z])([A-Z])', r'\1 \2', event1).title()
        hours_str = f"{rem_hours:.1f}"
        if rem_sec < max_sec:
            self.publish_event('SUN','INFO',event.upper(),sun)

        summary = f"Sun: {rem_hours:.1f} hours until {event}"
        diffs = self.num_string_differences(summary,self.sun_summary)
        if diffs > 7: # either startup, or transition to next event
            self.get_logger().warning(summary)
        elif diffs > 1:  # hourly update
            self.publish_event('SUN','INFO',summary,sun)
        self.sun_summary = summary


    def num_string_differences(self, s1, s2):
        """
        Returns the number of differences between two strings.
        """
        count = sum(1 for a, b in zip(s1, s2) if a != b) + abs(len(s1) - len(s2))
        return count


    def scene_callback(self, msg):
        """
        Callback function to handle incoming scene messages.
        """
        m = json.loads(msg.data)
        max_sec = m['interval']
        scene = m['payload']
        if m['index'] == 0:
            self.publish_event('SCENE','ERROR','Scene node restarted',scene)
        rem_hours = scene['secs_remaining'] / 3600.0
        scenename = scene['next_scene']    
        self.get_logger().info('Scene: %.1f hours until %s' % (rem_hours,scenename))
        if scene['secs_remaining'] <= max_sec:
            self.publish_event('SCENE','INFO','%s'% scenename,scene)


    def forecast_callback(self, msg):
        """
        Callback function to handle incoming weather forecast messages.
        """  
        m = json.loads(msg.data)
        wx = m['payload']  # a list of forecast dicts
        num_forecasts = len(self.forecasts)
        for forecast in wx:
            self.add_forecast(forecast) # check for new ones in here, redundats are dropped
        num_new_forecasts = len(self.forecasts) - num_forecasts
        # send the first one
        if num_forecasts == 0 and num_new_forecasts > 0:
            # publish most current forecast
            self.publish_forecast_event(self.forecasts[0][1])    
        # if we got a bunch summarize
        if num_new_forecasts >= 3:
            self.publish_forecast_summary_event()


    def add_forecast(self,forecast:dict):
        """
        Adds a new forecast to the forecast list.
        """
        now = datetime.datetime.now().isoformat()
        expiry_timestamp = forecast['endTime']
        if len(self.forecasts) == 0:
            self.get_logger().warning(f"First new forecast: {forecast["name"]} - {forecast["shortForecast"]}.")
            self.forecasts.append([expiry_timestamp,forecast])
        elif expiry_timestamp <= self.forecasts[-1][0]:
            # replace it
            # self.get_logger().warning(f"Updated forecast for {forecast["name"]}, replacing.")
            new_forecasts = []
            got_it = False
            for f_tuple in self.forecasts:
                if f_tuple[0] == expiry_timestamp:
                    new_forecasts.append([expiry_timestamp,forecast])
                    got_it = True
                else:
                    new_forecasts.append(f_tuple)
            if not got_it:
                if expiry_timestamp > now:
                    self.get_logger().warning(f"Forecast for {forecast["name"]} was missing, added.")
                    new_forecasts.append([expiry_timestamp,forecast])
                    new_forecasts.sort()
                else:
                    self.get_logger().warning(f"Forecast for {forecast["name"]} is expired, not added.")
            self.forecasts = new_forecasts
        else: # newer in time than any forecast we've got
            self.forecasts.append([expiry_timestamp,forecast])
            self.get_logger().info(f"added forcast for {forecast['name']} [exp:{forecast['endTime']}] len={len(self.forecasts)}")


    def publish_forecast_event(self, forecast):
        """
        Publishes a forecast event.
        """
        self.publish_event('FORECAST','INFO', f"{forecast['name']} - {forecast['detailedForecast']}", forecast) 
    

    def publish_forecast_summary_event(self):
        """
        Publishes a summary of the forecast events.
        """
        if len(self.forecasts) == 2:
            self.publish_forecast_event(self.forecasts[0][1])
            self.publish_forecast_event(self.forecasts[1][1])
        else:
            summary = "Summary: "
            for f_tuple in self.forecasts:
                summary += f_tuple[1]["name"] + " - "
                summary += f_tuple[1]["shortForecast"] + "; "
            if len(summary) > 2:
                summary = summary[:-2]
            self.publish_event('FORECAST','INFO',summary,self.forecasts)


    def conditions_callback(self, msg):
        """
        Callback function to handle incoming weather conditions messages.
        """
        m = json.loads(msg.data)
        wx = m['payload']
        station_id = wx['id']
        summary = wx['summary']
        observations = wx['observations']
        if station_id in self.wx_latest_conditions:
            if summary != self.wx_latest_conditions[station_id]:
                self.publish_event('CONDITIONS','INFO',summary,observations)
                self.wx_latest_conditions[station_id] = summary
        else:
            self.publish_event('CONDITIONS','INFO',summary,observations)
            self.wx_latest_conditions[station_id] = summary


    def wx_alerts_callback(self, msg):
        """
        Callback function to handle incoming weather alerts messages.
        """
        m = json.loads(msg.data)
        wx = m['payload']
        #self.publish_event('WEATHER','INFO','NEW ALERT: %s - %.1fF'% (wx['textDescription'],wx['temperature']['value']*9/5+32),wx)
        self.get_logger().info('Alert: %s' % msg.data)


    def node_list_callback(self,msg):
        """
        Callback function to handle incoming node list messages.
        """
        m = json.loads(msg.data)
        node_list = m['payload']
        # add new nodes
        for node_pid in node_list:
            if node_pid not in self.known_nodes:
                # add it
                node_name = node_list[node_pid]['name']
                self.known_nodes[node_pid] = node_list[node_pid]
                self.get_logger().warning(f"New ROS HOME node found: %s" % str(node_pid))
        # find lost nodes
        lost_nodes=[]
        for known in self.known_nodes:
            if known not in node_list:
                self.get_logger().error(f"ROS HOME node 'stopped running': %s" % str(known))
                self.publish_event('NODE','ERROR',f"ROS HOME node %s 'stopped running'!" % str(known), self.known_nodes[known])
                # TODO: restart the node, or do something else sensible
                lost_nodes.append(known)
        for lost in lost_nodes:
            del self.known_nodes[lost]


    def lighting_status_callback(self,msg):
        """
        Callback function to handle incoming lighting status messages.
        """
        m = json.loads(msg.data)         
        new_count = 0
        change_count = 0
        changes={}
        missing_count = 0
        num_reported = 0
        status = m['payload']
        id_base = status['type']+'/'+status['hub_ip'].replace('.','_')
        lights = status['lights']
        # any new lights?
        new_lights=[]
        for light in lights:
            light_id = light['room'].replace(' ','_') + '/' + id_base + '/' + 'id_' + light['id']
            if light_id not in self.lighting_status:
                new_count += 1
                self.lighting_status[light_id] = light
        if new_count > 0 :        
            self.publish_event('LIGHTS','INFO',f"Found {new_count} new lights on {id_base}", status)
        # any light status changes?
        for light in lights:
            num_reported += 1
            light_id = light['room'].replace(' ','_') + '/' + id_base + '/' + 'id_' + light['id']
            old_status = self.lighting_status[light_id]['state']
            new_status = light['state']
            if self.lighting_status[light_id]['state'] != light['state']:
                change_count += 1
                if light['room'] not in changes:
                    newly_on = 0
                    newly_off = 0
                    newly_brighter = 0
                    newly_dimmer = 0
                else:
                    newly_on = changes[light['room']]['on']
                    newly_off = changes[light['room']]['off']
                    newly_brighter = changes[light['room']]['brighter']
                    newly_dimmer = changes[light['room']]['dimmer']
                    
                if new_status == 0:    
                    newly_off+=1
                elif old_status == 0:
                    newly_on +=1
                elif new_status > old_status:
                    newly_brighter+=1
                else:
                    newly_dimmer+=1
                changes[light['room']] = {'on':newly_on,'off':newly_off,'brighter':newly_brighter,'dimmer':newly_dimmer}                    
            # remember light for next time        
            self.lighting_status[light_id] = light
        # report
        room_list = []
        for room in changes:
            room_list.append(room)
        if len(room_list) > 0:
            self.publish_event('LIGHTS','INFO',f"Lights were adjusted in {', '.join(room_list)}.",status)
            
        # any lights missing?
        self.get_logger().info(f"Lights: {num_reported} of {len(self.lighting_status)} total lights updated from {id_base}")     

        
    def detect_device_event(self):
        """
        Detects and handles device events based on the current and known devices.
        """
        known_len = len(self.known_devices)
        new_len = len(self.new_devices)
        if known_len == 0:
            self.publish_event('DEVICE','INFO','Devices: %d NEW devices' % new_len,self.new_devices)
        for d in self.new_devices:
            dev_is_new = True
            for kd in self.known_devices:
                if d['ip'] == kd['ip']:
                    kd['times_seen'] = kd['times_seen'] + 1
                    kd['last_seen'] = datetime.datetime.utcnow().isoformat()
                    dev_is_new = False
                    # TODO: merge the device data, preserving extra keys, known over 'Unknown' values
            if dev_is_new:
                d['times_seen'] = 1
                d['times_missed'] = 0
                d['last_seen'] = datetime.datetime.utcnow().isoformat()
                self.known_devices.append(d)
                self.publish_event('DEVICE','INFO','Devices: NEW device: %s@%s' % (d['name'],d['ip']),d)
        # check if a known device is missing from the lastest scan        
        for kd in self.known_devices:
            dev_is_lost = True
            for d in self.new_devices:
                if d['ip'] == kd['ip']:
                    dev_is_lost = False
            if dev_is_lost:
                kd['times_missed'] = kd['times_missed'] + 1
                self.publish_event('DEVICE','WARNING','Devices: Missed known device: %s@%s last seen %s' % (kd['name'],kd['ip'],kd['last_seen']),kd)
        # share the master list        
        self.publish_known_devices()


# Main entry point
def main(args=None):
    # Initialize the ROS node
    rclpy.init(args=args)
    # Create the EventDetector node
    home_event_detect = EventDetector()
    # Spin the node
    rclpy.spin(home_event_detect)
    # Destroy the node   
    home_event_detect.destroy_node()
    rclpy.shutdown()

# Main entry point
if __name__ == '__main__':
    main()

# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#

from cmath import nan
from distutils.log import set_verbosity
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import json
import datetime

class EventDetector(Node):

    def __init__(self):
        super().__init__('home_event_detect')
        self.lighting_status={}
        self.new_devices=[]
        self.known_devices=[]
        self.known_nodes={}
        self.forecasts=[]
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
        self.i += 1
        self.prune_forecast_list()


    def prune_forecast_list(self):
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
            self.publish_forecast_event(new_forecasts[0][0])
        self.forecasts = new_forecasts


    def json_commands_callback(self,msg):
        m = json.loads(msg.data)
        # self.device_interval = m['interval']
        command = m['payload']
        #if m['index'] == 0:
        #    self.publish_event('DEV','ERROR','Device discovery node restarted',self.new_devices)
        if "argv" in command:
            argv = command['argv']
            self.parse_command(argv)


    def raw_commands_callback(self,msg):
        command = msg.data
        argv=command.split()
        self.parse_command(argv)


    def parse_command(self,argv):
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
        msg = String()
        event = {}
        event['event_type'] = typename
        event['severity'] = severity
        event['timestamp'] = datetime.datetime.utcnow().isoformat()
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
        elif severity=='WARNING':
            self.get_logger().warn(desc)
        else:
            self.get_logger().info(desc)
        self.event_index += 1    


    def publish_known_devices(self):
        msg = String()
        msgdict = {}
        msgdict['index'] = self.i
        msgdict['interval'] = self.device_interval
        msgdict['payload'] = self.known_devices
        msg.data = json.dumps(msgdict)
        self.publisher_devices.publish(msg)
        self.get_logger().info('Devices: Publishing known devices list: %d devices' % len(self.known_devices))


    def devices_callback(self, msg):
        m = json.loads(msg.data)
        self.device_interval = m['interval']
        self.new_devices = m['payload']
        #if m['index'] == 0:
        #    self.publish_event('DEV','ERROR','Device discovery node restarted',self.new_devices)
        self.detect_device_event()


    def owntracks_callback(self, msg):
        self.get_logger().info('Owntracks: "%s"' % msg.data)
        md = json.loads(msg.data)
        track = md['payload']
        if track['type'] == 'EVENT':
            self.publish_event('TRACK','INFO',track['description'],track)


    def sun_callback(self, msg):
        m = json.loads(msg.data)
        max_sec = m['interval']
        sun = m['payload']
        #if m['index'] == 0:
        #    self.publish_event('/environment/sun','ERROR','Sun tracker node restarted',sun)
        rem_hours = sun['secs_remaining'] / 3600.0
        event = sun['next_event']    
        self.get_logger().info('Sun: %.1f hours until %s' % (rem_hours,event))
        if sun['secs_remaining'] <= max_sec:
            self.publish_event('SUN','INFO','%s'% event,sun)


    def scene_callback(self, msg):
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
        expiry_timestamp = forecast['endTime']
        if len(self.forecasts) == 0:
            self.get_logger().warning(f"First new forecast: {forecast["name"]} - {forecast["shortForecast"]}.")
            self.forecasts.append([expiry_timestamp,forecast])
        elif expiry_timestamp <= self.forecasts[-1][0]:
            # replace it
            self.get_logger().warning(f"Updated forecast for {forecast["name"]}, replacing.")
            new_forecasts = []
            got_it = False
            for f_tuple in self.forecasts:
                if f_tuple[0] == expiry_timestamp:
                    new_forecasts.append([expiry_timestamp,forecast])
                    got_it = True
                else:
                    new_forecasts.append(f_tuple)
            if not got_it:
                self.getlogger().warning(f"Forecast for {forecast["name"]} was missing, added.")
                new_forecasts.append([expiry_timestamp,forecast])
                new_forecasts.sort()
            self.forecasts = new_forecasts
        else: # newer in time than any forecast we've got
            self.forecasts.append([expiry_timestamp,forecast])
            self.get_logger().info(f"added forcast for {forecast['name']} [exp:{forecast['endTime']}] len={len(self.forecasts)}")


    def publish_forecast_event(self, forecast):
        self.publish_event('FORECAST','INFO', f"{forecast['name']} - {forecast['detailedForecast']}", forecast) 
    

    def publish_forecast_summary_event(self):
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
        m = json.loads(msg.data)
        wx = m['payload']
        station_id = wx['id']
        summary = wx['summary']
        observations = wx['observations']
        if station_id in self.wx_latest_conditions:
            if summary != self.wx_alerts_callback[station_id]:
                self.publish_event('CONDITIONS','INFO',summary,observations)
                self.wx_latest_conditions[station_id] = summary
        else:
            self.publish_event('CONDITIONS','INFO',summary,observations)
            self.wx_latest_conditions[station_id] = summary


    def wx_alerts_callback(self, msg):
        m = json.loads(msg.data)
        wx = m['payload']
        #self.publish_event('WEATHER','INFO','NEW ALERT: %s - %.1fF'% (wx['textDescription'],wx['temperature']['value']*9/5+32),wx)
        self.get_logger().info('Alert: %s' % msg.data)


    def node_list_callback(self,msg):
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

def main(args=None):
    rclpy.init(args=args)

    home_event_detect = EventDetector()

    rclpy.spin(home_event_detect)

    home_event_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

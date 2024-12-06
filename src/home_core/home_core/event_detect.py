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
        self.new_devices=[]
        self.known_devices=[]
        self.known_nodes={}
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

        self.publisher_events = self.create_publisher(String, '/diagnostics/events', 10)
        self.period_name = ""
        self.timer_period = 10.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.dev_index = 0
        self.event_index = 0
        
        self.publisher_devices = self.create_publisher(String, '/devices/known/network', 10)
        self.wx_latest_conditions = ""
        self.wx_recent_forecasts = {}
        self.lights = []
        self.num_lights_on = -1


    def timer_callback(self):
        self.i += 1


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
            self.publish_event('/environment/sun','INFO','%s'% event,sun)


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
        max_sec = m['interval']
        wx = m['payload']
        if m['index'] == 0:
            self.publish_event('FORECAST','ERROR','Weather tracker node restarted',wx)
            self.period_name = ""
        if self.period_name != wx[0]['name']:
            self.publish_event('FORECAST','INFO','%s - %s'% (wx[0]['name'],wx[0]['detailedForecast']),wx)
            self.period_name = wx[0]['name']
        else:
            self.get_logger().info('Forecast: %s - %s' % (wx[0]['name'],wx[0]['detailedForecast']))


    def conditions_callback(self, msg):
        m = json.loads(msg.data)
        wx = m['payload']
        if wx['temperature']['value']:        
            if (type(wx['temperature']['value']) == int) or (type(wx['temperature']['value']) == float):
                conditions_desc = '%s - %.1fF' % (wx['textDescription'],wx['temperature']['value']*9/5+32)
                if conditions_desc != self.wx_latest_conditions:   # only publish a conditions event if the conditions have changed
                    self.wx_latest_conditions = conditions_desc
                    self.publish_event('CONDITIONS','INFO',conditions_desc,wx)
                else: 
                    self.get_logger().info('Conditions: %s - %.1fF' % (wx['textDescription'],wx['temperature']['value']*9/5+32))
            else:
                self.get_logger().warning('Conditions: %s - %s (weird temp)' % (wx['textDescription'],wx['temperature']['value']))
        else:
            self.get_logger().warning('Conditions: %s - (no temp)' % (wx['textDescription']))


    def wx_alerts_callback(self, msg):
        m = json.loads(msg.data)
        wx = m['payload']
        #self.publish_event('WEATHER','INFO','NEW ALERT: %s - %.1fF'% (wx['textDescription'],wx['temperature']['value']*9/5+32),wx)
        self.get_logger().info('Alert: %s' % msg.data)


    def node_list_callback(self,msg):
        m = json.loads(msg.data)
        node_list = m['payload']
        # add new nodes
        for node in node_list:
            if node not in self.known_nodes:
                # add it
                self.known_nodes[node] = node_list[node]
                self.get_logger().warning(f"New ROS HOME node found: %s" % str(node))
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
        lights = m['payload']['lights']
        num_on = 0 
        num_lights = len(lights)
        for light in lights:
            if light['state'] != 0:  
                num_on += 1
        if self.num_lights_on != num_on:   # FIXME: better change detect on light status       
            if num_on == num_lights:
                self.publish_event('LIGHTS','WARNING',f"All %s lights are on" % m['payload']['type'],lights)
            if num_on == 0:
                self.publish_event('LIGHTS','INFO',f"All %s lights are off" % m['payload']['type'],lights)
        self.get_logger().info(f"Lights: %d of %d lights on from %s" %(num_on,num_lights,m['payload']['type']))    
        self.lights = lights  # FIXME: merge each lighting status message into master list    
        self.num_lights_on = num_on
                
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

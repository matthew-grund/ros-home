# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import json
import paho.mqtt.client as mqtt
import urllib.parse
from math import nan

owntracks_reader = None 
class OwnTracksMQTTReader(Node):
    
    def __init__(self):
        super().__init__('owntracks_read')
        self.publisher_tracks = self.create_publisher(String, '/people/tracking/owntracks', 10)
        #self.msg_period = 10.0  # seconds
        #self.host_timer = self.create_timer(self.msg_period, self.mqtt_poll_callback)
        self.msg_index  = 0

        # subscribe to settings
        self.settings_sub = self.create_subscription(
            String,
            '/home/configuration',
            self.settings_callback,
            10)
        self.settings_sub  # prevent unused variable warning
        
        self.need_mqtt_settings = True
        self.need_mqtt_connect = True
        
        self.last_event_time = 0
        self.last_track_time = 0
        self.last_region = "Earth"
        
    def settings_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "OWNTRACKS":
            self.mqtt_server = msg['payload']
            self.get_logger().info(f"Got owntracks mqtt server: %s" % json.dumps(self.mqtt_server))
            self.url_str = self.process_url(self.mqtt_server)
            self.need_mqtt_settings = False
            self.get_logger().info(f"Got config: URL is: %s" % self.url_str )

    def process_url(self,mqtt_cfg):
        server = mqtt_cfg['Server']['name']
        port = mqtt_cfg['Server']['port']
        self.topic = mqtt_cfg['Server']['topic']
        protocol = mqtt_cfg['Server']['protocol']
        username = mqtt_cfg['User']['username']
        password = mqtt_cfg['User']['password']
        url_str = protocol + "://" + username + ":" + password + "@" + server + ":" + port
        return url_str

    def publish_event(self,event):
        owntracks_reader.get_logger().info(f"Publishing new event: %s" % json.dumps(event))
        e = {}
        e['type'] = "EVENT"
        e['timestamp'] = event['tst']
        e['latitude_deg'] = event['lat']
        e['longitude_deg'] = event['lon']
        e['tracked_id'] = event['tid'].upper()
        if event['event'] == 'leave':
            e['action'] = 'EXFILTRATE'
            dir = " Departed "
        else:
            e['action'] = 'INFILTRATE'
            dir = " Arrived "
        e['region'] = event['desc']    
        e['description'] = event['tid'].upper() + dir + event['desc']
        self.publish_msg(e)
        self.last_event_time = event['tst']
        self.last_region = e['region']
        
    def publish_track(self,track):
        owntracks_reader.get_logger().info(f"Publishing new track: %s" % json.dumps(track))
        t = {}
        t['type'] = "TRACK"
        t['timestamp'] = track['tst']
        t['latitude_deg'] = track['lat']
        t['longitude_deg'] = track['lon']
        t['tracked_id'] = track['tid'].upper()
        t['phone_battery_percent'] = track['batt'] 
        if 'cog' in track:
            t['course_deg'] = track['cog']
        else:
            t['course_deg'] = nan
        if 'vel' in track:   
            t['speed_kph'] = track['vel']   
        else:
            t['speed_kph'] = 0
        if 'inregions' in track:
            t['region'] = track['inregions'][0]
        else:
            # "SSID": "9C Riverview"
            if 'SSID' in track:
                if track['SSID'] == '9C Riverview':
                    t['region'] = 'Home'
                elif track['SSID'] == 'eduroam': 
                    t['region'] = 'Work'
                else:        
                    t['region'] = 'Earth'   
            else:        
                t['region'] = 'Earth'  
        
        # perhaps send an event instead?
        if (self.last_region != t['region']) and ((int(track['tst']) - int(self.last_event_time)) > 180) :
            t['type'] = "EVENT"
            if t['region'] == 'Earth':
                t['action'] = 'EXFILTRATE'
                dir = " Departed "
                action_region = self.last_region
            else:
                t['action'] = 'INFILTRATE'
                dir = " Arrived "
                action_region = t['region']
            t['description'] = track['tid'].upper() + dir + action_region       
            self.last_event_time = track['tst']
        else:
            self.last_track_time = track['tst']
        self.last_region = t['region']
            
        # ship it            
        self.publish_msg(t)
        
    def publish_msg(self,payload_dict):
        msg = String()
        msgdict = {}
        msgdict['index'] = self.msg_index
        self.msg_index += 1
        msgdict['interval'] = nan
        msgdict['payload'] = payload_dict
        msg.data = json.dumps(msgdict)
        self.publisher_tracks.publish(msg)
        self.get_logger().info(f"Published: %s" % msg.data)

    
def on_connect(client, userdata, flags, rc):
        global owntracks_reader
        if owntracks_reader == None:
            print(f"MQTT Connected: %s" % str(rc))
        else:
            owntracks_reader.get_logger().info(f"Connected to MQTT Server: %s" % str(rc))
            
def on_message(client, obj, msg):
        global owntracks_reader
        if owntracks_reader == None:
            print(f"MQTT New track: %s" % str(msg.payload))
        else:
            track = json.loads(msg.payload)
            if 'event' in track:
                owntracks_reader.get_logger().info(f"MQTT Got event: %s" % str(msg.payload))
                timestamp = track['tst']
                if timestamp > owntracks_reader.last_event_time:
                    owntracks_reader.publish_event(track)
            else:
                if track['_type'] == 'location':
                    owntracks_reader.get_logger().info(f"MQTT Got track: %s" % str(msg.payload))
                    timestamp = track['tst']
                    if timestamp > owntracks_reader.last_track_time:
                        owntracks_reader.publish_track(track)
            
def on_publish(client, obj, mid):
        global owntracks_reader
        if owntracks_reader == None:
            print(f"MQTT Sent: %s" % str(mid))
        else:
            owntracks_reader.get_logger().info(f"MQTT Sent: %s" % str(mid))
            
def on_subscribe(client, obj, mid, granted_qos):
        global owntracks_reader
        if owntracks_reader == None:
            print(f"MQTT Subscribed: %s" % str(mid) + " " + str(granted_qos))
        else:
            owntracks_reader.get_logger().info(f"MQTT Subscribed: %s" % str(mid) + " " + str(granted_qos))

def on_log(client, obj, level, string):
        global owntracks_reader
        if owntracks_reader == None:
            print(f"MQTT => %s" % string)
        else:
            owntracks_reader.get_logger().debug(f"MQTT: %s" % string)


def main(args=None): 
          
    rclpy.init(args=args)
    
    #the  mqtt client
    mqtt_client = mqtt.Client()

    # Assign event callbacks
    mqtt_client.on_message = on_message
    mqtt_client.on_connect = on_connect
    mqtt_client.on_publish = on_publish
    mqtt_client.on_subscribe = on_subscribe
    mqtt_client.on_log = on_log
    
    # the ROS node
    global owntracks_reader 
    owntracks_reader = OwnTracksMQTTReader()

    while owntracks_reader.need_mqtt_connect:
        rclpy.spin_once(owntracks_reader)
        if owntracks_reader.need_mqtt_settings == False:
            if owntracks_reader.need_mqtt_connect:
                owntracks_reader.get_logger().info(f"Connecting to: %s" % owntracks_reader.url_str )
                url = urllib.parse.urlparse(owntracks_reader.url_str)
                # Connect
                mqtt_client.username_pw_set(url.username, url.password)
                mqtt_client.connect(url.hostname, url.port)
                # Start subscribe, with QoS level 0
                mqtt_client.subscribe(owntracks_reader.topic, 0)
                mqtt_client.subscribe(owntracks_reader.topic + "/event", 0)
                owntracks_reader.need_mqtt_connect = False
                mqtt_client.loop_start() # spawns a thread to handle MQTT  
                       
    rclpy.spin(owntracks_reader)
    
    owntracks_reader.destroy_node()
    rclpy.shutdown()


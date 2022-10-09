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
 
class OwnTracksMQTTReader(Node):
    
    def __init__(self):
        super().__init__('owntracks_read')
        self.publisher_tracks = self.create_publisher(String, 'owntracks', 10)
        #self.msg_period = 10.0  # seconds
        #self.host_timer = self.create_timer(self.msg_period, self.mqtt_poll_callback)
        self.msg_index  = 0

        # subscribe to settings
        self.settings_sub = self.create_subscription(
            String,
            'settings',
            self.settings_callback,
            10)
        self.settings_sub  # prevent unused variable warning
        
        self.need_mqtt_settings = True
        self.need_mqtt_connect = True
        
        #the  mwtt client
        self.mqtt_client = mqtt.Client()

        # Assign event callbacks
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_publish = self.on_publish
        self.mqtt_client.on_subscribe = self.on_subscribe

    def on_connect(self,client, userdata, flags, rc):
        # print("connect: " + str(rc))
        self.get_logger().info(f"Connect: %s" % str(rc))

    def on_message(self,client, obj, msg):
        # print("message: " + msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        self.get_logger().info(f"New track: %s" % str(msg.payload))
        
    def on_publish(self,client, obj, mid):
        # print("mid: " + str(mid))
        self.get_logger().info(f"Sent: %s" % str(mid))

    def on_subscribe(self,client, obj, mid, granted_qos):
        # print("Subscribed: " + str(mid) + " " + str(granted_qos))
        self.get_logger().info(f"Subscribed: %s" % str(mid) + " " + str(granted_qos))

    def settings_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "OWNTRACKS":
            self.mqtt_server = msg['payload']
            self.get_logger().info(f"Got owntracks mqtt server: %s" % json.dumps(self.mqtt))
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
 
    def connect_mqtt(self):
        url = urllib.parse.urlparse(self.url_str)
        # Connect
        self.mqtt_client.username_pw_set(url.username, url.password)
        self.mqrr_client.connect(url.hostname, url.port)
        # Start subscribe, with QoS level 0
        self.mqtt_client.subscribe(self.topic, 0)
 
def main(args=None):       
    rclpy.init(args=args)

    owntracks_reader = OwnTracksMQTTReader()

    # rclpy.spin(owntracks_reader)
    mqtt_ret = 0
    while mqtt_ret == 0:
        rclpy.spin_once(owntracks_reader)
        if owntracks_reader.need_mqtt_settings == False:
            if owntracks_reader.need_mqtt_connect == False:
                mqtt_ret = owntracks_reader.mqtt_client.loop()
            else:
                owntracks_reader.connect_mqtt()                   

    owntracks_reader.destroy_node()
    rclpy.shutdown()

        
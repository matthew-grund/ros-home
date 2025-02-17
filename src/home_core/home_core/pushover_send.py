# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import http.client, urllib
import json
import datetime

class PushoverMessager(Node):

    def __init__(self):
        super().__init__('pushover_messager')
        self.publisher_commands = self.create_publisher(String,'/home/commands/json', 10)
        self.event_subscription = self.create_subscription(
            String,
            '/home/events',
            self.event_listener_callback,
            10)
        self.event_subscription  # prevent unused variable warning
        self.config_subscription = self.create_subscription(
            String,
            '/home/configuration',
            self.config_listener_callback,
            10)
        self.config_subscription  # prevent unused variable warning
        self.get_logger().info(f"Waiting for config message on topic '/home/configuration'")
        self.timer_period = 15.0
        self.ticks_elapsed = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.need_config = True


    def timer_callback(self):
        if self.need_config:
            self.ticks_elapsed += 1
            if self.ticks_elapsed >2:
                m = {}
                m['index'] = self.ticks_elapsed
                m['interval'] = self.timer_period
                m['payload'] = {"argv":["configure","pushover"]}
                msg = String()
                msg.data = json.dumps(m)  # ask the configuration thing to send pushover config
                self.publisher_commands.publish(msg)
                self.get_logger().warning(f"No config after {self.timer_period * self.ticks_elapsed} sec. sending command.")


    def event_listener_callback(self, msg):
        if not self.need_config:
            m = json.loads(msg.data)
            event = m['payload']
            subj = event['event_type']
            msg = event['description']
            self.send(subj,msg)


    def config_listener_callback(self, msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "PUSHOVER":
            self.user_token = msg['payload']['Account']['user_token'] 
            self.default_token = msg['payload']['Apps']['default_app_token']
            self.daylight_token = msg['payload']['Apps']['daylight_app_token']
            self.lighting_token = msg['payload']['Apps']['lighting_app_token']
            self.env_token = msg['payload']['Apps']['environment_app_token']
            self.diagnostics_token = msg['payload']['Apps']['diagnostics_app_token']
            self.need_config = False
            self.get_logger().info(f"Got PUSHOVER messager configuration")
            # TODO - add people config

            
    def app_token_from_message(self, subject, content):
        if subject == 'CONDITIONS':
            token = self.env_token
        elif subject == 'FORECAST':
            token = self.env_token
        elif subject == 'LIGHTS':
            token = self.lighting_token
        elif subject == 'SCENE':
            token = self.diagnostics_token
        elif subject == 'SUN':
            token = self.daylight_token
        else:
            token = self.default_token
        return token
            

    def send(self, subject, content):
        now = datetime.datetime.now()
        now_str = now.strftime("[%H:%M] ")
        full_content = now_str + content

        token = self.app_token_from_message(subject, content)
        
        conn = http.client.HTTPSConnection("api.pushover.net:443")

        conn.request("POST", "/1/messages.json",
                          urllib.parse.urlencode({
                              "token": token,
                              "user": self.user_token,
                              "title": subject,
                              "ttl": 3600 * 4.0,
                              "message": full_content,
                          }), { "Content-type": "application/x-www-form-urlencoded" })
        conn.getresponse()

        self.get_logger().info(f"Pushover sent {subject} {content}")
        

def main(args=None):
    rclpy.init(args=args)

    pushover_messager = PushoverMessager()

    rclpy.spin(pushover_messager)

    pushover_messager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

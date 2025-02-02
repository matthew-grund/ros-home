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

        self.need_config = True


    def event_listener_callback(self, msg):
        if not self.need_config:
            m = json.loads(msg.data)

            event = m['payload']
            subj = event['event_type']
            msg = event['description']

            self.send(subj,msg)


    def config_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "PUSHOVER":
            self.user_token = msg['payload']['Account']['user_token'] 
            self.default_token = msg['payload']['Apps']['default_app_token']
            self.env_token = msg['payload']['Apps']['environment_app_token']
            self.need_config = False
            self.get_logger().info(f"Got PUSHOVER messager configuration")
            # TODO - add people config


    def send(self, subject, content):
        now = datetime.datetime.now()
        now_str = now.strftime("[%H:%M] ")
        content=now_str+content
        conn = http.client.HTTPSConnection("api.pushover.net:443")

        if subject=='CONDITIONS':
            token = self.env_token
        elif subject=='FORECAST':
            token = self.env_token
        else:
            token = self.default_token

        conn.request("POST", "/1/messages.json",
                          urllib.parse.urlencode({
                              "token": token,
                              "user": self.user_token,
                              "title": subject,
                              "message": content,
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

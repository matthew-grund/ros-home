# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smtplib
import ssl
import json
import datetime
class SMSMessager(Node):

    def __init__(self):
        super().__init__('sms_messager')
        self.event_subscription = self.create_subscription(
            String,
            'events',
            self.event_listener_callback,
            10)
        self.event_subscription  # prevent unused variable warning
        self.config_subscription = self.create_subscription(
            String,
            'settings',
            self.config_listener_callback,
            10)
        self.config_subscription  # prevent unused variable warning
        self.need_config = True

    def event_listener_callback(self, msg):
        if not self.need_config:
            m = json.loads(msg.data)
            event = m['payload']
            subj = event['event_type']
            msg = event['description']
            # filter the events
            if subj!="DEVICE":          # device discovery very noisy at the minute
                self.send(subj,msg)
    
    def config_listener_callback(self,msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "SMS":
            self.port = msg['payload']['Server']['port'] 
            self.smtp_server_domain_name = msg['payload']['Server']['smtp_server']
            self.sender_mail = msg['payload']['Sender']['address']
            self.password = msg['payload']['Sender']['password']
            self.dest_mail = msg['payload']['Recipient']['address']
            self.need_config = False
            self.get_logger().info(f"Got config: {self.sender_mail} sending to {self.dest_mail}")
            # TODO - add people config

    def send(self, subject, content):
        now = datetime.datetime.now()
        now_str = now.strftime("%H%M")
        ssl_context = ssl.create_default_context()
        service = smtplib.SMTP_SSL(self.smtp_server_domain_name, self.port, context=ssl_context)
        service.login(self.sender_mail, self.password)
        result = service.sendmail(self.sender_mail, self.dest_mail, f"Subject: {subject}\n[{now_str}]  {content}")
        service.quit()
        self.get_logger().info(f"Sending {subject}:{content}")


def main(args=None):
    rclpy.init(args=args)

    sms_messager = SMSMessager()

    rclpy.spin(sms_messager)

    sms_messager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

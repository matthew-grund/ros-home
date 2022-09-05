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
class SMSMessager(Node):

    def __init__(self):
        super().__init__('sms_messager')
        self.event_subscription = self.create_subscription(
            String,
            'events',
            self.event_listener_callback,
            10)
        self.event_subscription  # prevent unused variable warning
        self.port = 465
        self.smtp_server_domain_name = "smtp.gmail.com"
        self.sender_mail = "9criverview@gmail.com"
        self.password = "zegvbejqjvcbvdin"
        self.dest_mail = "5085244628@mms.att.net"


    def event_listener_callback(self, msg):
        m = json.loads(msg.data)
        event = m['payload']
        subj = event['event_type']
        msg = event['description']
        if subj!="DEV":
            self.send(subj,msg)
    
    def send(self, subject, content):
        ssl_context = ssl.create_default_context()
        service = smtplib.SMTP_SSL(self.smtp_server_domain_name, self.port, context=ssl_context)
        service.login(self.sender_mail, self.password)
        result = service.sendmail(self.sender_mail, self.dest_mail, f"Subject: {subject}\n{content}")
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

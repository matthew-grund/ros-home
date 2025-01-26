# Copyright 2022 Matthew Grund
#
# Licensed under the BSD 2 Clause license;
# you may not use this file except in compliance with the License.
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests

class NestThermostat:
    """
    A class to interface with a Nest thermostat.

    Attributes:
        access_token (str): The access token for authenticating with the Nest API.
        device_id (str): The ID of the Nest thermostat device.
        base_url (str): The base URL for the Nest API.

    Methods:
        get_status(): Retrieves the current status of the thermostat.
        set_temperature(temp): Sets the target temperature of the thermostat.
    """

    def __init__(self, access_token, device_id):
        self.access_token = access_token
        self.device_id = device_id
        self.base_url = 'https://developer-api.nest.com'

    def get_status(self):
        url = f'{self.base_url}/devices/thermostats/{self.device_id}'
        headers = {
            'Authorization': f'Bearer {self.access_token}',
            'Content-Type': 'application/json'
        }
        response = requests.get(url, headers=headers)
        if response.status_code == 200:
            return response.json()
        else:
            response.raise_for_status()

    def set_temperature(self, temp):
        url = f'{self.base_url}/devices/thermostats/{self.device_id}'
        headers = {
            'Authorization': f'Bearer {self.access_token}',
            'Content-Type': 'application/json'
        }
        data = {
            'target_temperature_f': temp
        }
        response = requests.put(url, headers=headers, json=data)
        if response.status_code == 200:
            return response.json()
        else:
            response.raise_for_status()


class NestController(Node):
    """
    A ROS2 Node that controls and publishes the status of a Nest thermostat.

    Attributes:
        publisher_ (Publisher): A ROS2 publisher that publishes HVAC status messages.
        timer (Timer): A ROS2 timer that triggers the timer_callback method periodically.
        i (int): A counter used in the timer_callback method.
        config_subscription (Subscription): A ROS2 subscription to listen for configuration messages.

    Methods:
        __init__(): Initializes the NestController node, sets up the publisher, timer, and subscription.
        timer_callback(): Callback function that publishes a status message periodically.
        publish_status(status): Publishes a given status message.
    """

    def __init__(self):
        super().__init__('nest_controller')
        self.nest_status_publisher = self.create_publisher(String, '/hvac/status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.config_subscription = self.create_subscription(
            String,
            '/home/configuration',
            self.config_listener_callback,
            10)
        self.config_subscription


    def config_listener_callback(self, msg):
        msg = json.loads(msg.data)
        if msg['payload']['type'] == "LUTRON":
            self.lutron_ipaddress = msg['payload']['Bridge']['address']
            self.lutron_certfile = msg['payload']['Certificate']['filepath']
            self.lutron_keyfile = msg['payload']['Key']['filepath']
            self.lutron_bridgecertfile = msg['payload']['Bridge Certificate']['filepath']
            self.do_need_config_msg = False
            self.get_logger().info(f"Got config: SmartBridge is {self.lutron_ipaddress}" )

    def timer_callback(self):
        msg = String()
        msg.data = 'Thermostat: %d' % self.i
        self.nest_status_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.nest_status_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    nest_controller = NestController()

    rclpy.spin(nest_controller)

    nest_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

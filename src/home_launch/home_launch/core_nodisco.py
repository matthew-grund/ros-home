from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

   # discovery_node = Node(
   #     package="home_core",
   #     executable="netdisco",
   # )

    weather_node = Node(
        package="home_core",
        executable="weather"
    )

    sun_node = Node(
        package="home_core",
        executable="sun"
    )

    event_node = Node(
        package="home_core",
        executable="event"
    )
 
    sms_node = Node(
        package="home_core",
        executable="sms"
    )

    config_node = Node(
        package="home_core",
        executable="config"
    )

    # ld.add_action(discovery_node)
    ld.add_action(weather_node)
    ld.add_action(sun_node)
    ld.add_action(event_node)
    ld.add_action(sms_node)
    ld.add_action(config_node)

    return ld

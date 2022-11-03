from launch import LaunchDescription
from launch_ros.actions import Node
#
#  The "home_core" package has several nodes that every ROS Home would use
#
def generate_launch_description():
    ld = LaunchDescription()
    
    # The config node monitors a directory for INI files, and publishes contents of those files
    # to the "settings" topic. This node is flexible, and will publish contents of ANY INI file.
    #
    config_node = Node(
        package="home_core",
        executable="config"
    )

    # network discovery of smart home devices
    discovery_node = Node(
        package="home_core",
        executable="netdisco",
    )

    # the core control triggering concept in ROS Home is the event. This node filters data from "track"
    # nodes, and develops events.
    event_node = Node(
        package="home_core",
        executable="event"
    )

    # MQTT interface to OwnTracks app
    owntracks_node = Node(
        package="home_core",
        executable="owntracks",
    )
 
    # The monitor node measures and reports environmental data, and performance of the ROS HOME system
    monitor_node = Node(
        package="home_core",
        executable="monitor",
    )

    # the schedule node reads a scene schedule INI, and publishes 
    # event-ready time data for the day
    schedule_node = Node(
        package="home_core",
        executable="schedule"
    )
    
    # The SMS node sits on the "events" topic, and notifies users by email, with 
    # SMS email addresses possible.
    sms_node = Node(
        package="home_core",
        executable="sms"
    )
    
    # sun position in the sky can inform control decisions,
    # like lighting activation times, with events published like "Sunrise".
    sun_node = Node(
        package="home_core",
        executable="sun"
    )

    # weather forecast and current conditions can inform 
    # control decisions, like thermostat setting
    weather_node = Node(
        package="home_core",
        executable="weather"
    )
    
    # lights via lutron_devices
    lutron_node = Node(
        package="home_devices",
        executable="lutron"
    )
    
    # media via yamaha receivers
    yamaha_node = Node(
        package="home_devices",
        executable="yamaha"
    )

    ld.add_action(discovery_node)
    ld.add_action(weather_node)
    ld.add_action(sun_node)
    ld.add_action(schedule_node)
    ld.add_action(event_node)
    ld.add_action(sms_node)
    ld.add_action(owntracks_node)
    ld.add_action(lutron_node)
    ld.add_action(yamaha_node)
    ld.add_action(monitor_node)
    ld.add_action(config_node)

    return ld

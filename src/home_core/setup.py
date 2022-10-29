from setuptools import setup

package_name = 'home_core'

setup(
    name=package_name,
    version='0.8.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Grund',
    maintainer_email='matthew.grund77@gmail.com',
    description='Home_core provides home device discovery and simple control',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command = home_core.command:main',
            'config = home_core.file_configure:main',
            'event = home_core.event_detect:main',
            'monitor = home_core.home_monitor:main', 
            'netdisco = home_core.net_discover:main',
            'btdisco = home_core.bt_discover:main',
            'observe = home_core.home_monitor:main',
            'occupancy = home_core.occupancy_track:main',
            'owntracks = home_core.owntracks_read:main',
            'scene = home_core.scene_execute:main',
            'schedule = home_core.scene_schedule:main',
            'sun = home_core.sun_track:main',
            'sms = home_core.sms_send:main',
            'weather = home_core.weather_track:main'
        ],
    },
)      


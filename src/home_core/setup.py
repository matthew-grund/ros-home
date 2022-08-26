from setuptools import setup

package_name = 'home_core'

setup(
    name=package_name,
    version='0.0.0',
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
    license='BSD-2-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'discover = home_core.discover:main',
            'event = home_core.event_detect:main',
            'command = home_core.command:main',
            'sun = home_core.sun_track:main',
            'schedule = home_core.schedule:main',
            'scene = home_core.scene_execute:main',
            'occupancy = home_core.occupancy_track:main',
            'weather = home_core.weather_track:main',
            'sms = home_core.sms_message:main'
        ],
    },
)      


from setuptools import setup

package_name = 'home_devices'

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
    description='Home devices provides ROS2 node interfaces to home devices',
    license='BSD-2-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch = home_devices.device_launch:main',
            'lutron = home_devices.lutron_device:main',
            'yamaha = home_devices.yamaha_device:main',      
            'nest = home_devices.nest_device:main',
            'google = home_devices.google_device:main',
            'xbox = home_devices.xbox_device:main',
            'lgtv = home_device.lgtv_device:main',                        
        ],
    },
)

from setuptools import setup
import os
from glob import glob

package_name = 'home_launch'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('home_launch/*_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matthew Grund',
    maintainer_email='"matthew.grund77@gmail.com"',
    description='Home_launch provides home node launch',
    license='BSD-2-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

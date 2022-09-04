from setuptools import setup

package_name = 'home_extras'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='"matthew.grund77@gmail.com"',
    description='Home_extras provides extra miscellaneous ROS2 node interfaces',
    license='BSD-2-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'event = home_extras.event_detect:main'
            'stocks = home_extras.stock_track:main',
            'tides = home_extras.tide_track:main',
            'nfl = home_extras.nfl_track:main',
            'f1 = home_extras.formula1_track:main',
            'mlb = home_extras.mlb_track:main',
        ],
    },
)

from setuptools import setup

package_name = 'home_ui'

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
    maintainer_email='"matthew.grund77@gmail.com"',
    description='Home UI provides configuration, introspection and monitoring tools for ROS Home',
    license='BSD-2-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'tui = home_ui.home_tui:main',
                'qtui = home_ui.home_qtui:main'
        ],
    },
)

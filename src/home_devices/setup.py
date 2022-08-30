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
    description='Home_devices provides a library of ros node interfaces to home devices',
    license='BSD-2-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lutron = home_devices.lutron:main',
            'yamaha = home_devices.yamaha:main'
        ],
    },
)      

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
    maintainer='matt',
    maintainer_email='"matthew.grund77@gmail.com"',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

from setuptools import setup

package_name = 'rvr_autokit_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/rvr_autokit.launch.py',
            'launch/fleet_logger.launch.py'
        ]),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
        'sparkfun-qwiic-vl53l1x',
        'sparkfun-qwiic-titan-gps',
        'pynmea2',
    ],
    zip_safe=True,
    maintainer='mik-p',
    maintainer_email='mppritchard3@hotmail.com',
    description='ROS 2 driver for Sphero RVR Sparkfun Autonomous Kit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sparkfun_autokit_driver = rvr_autokit_driver.sparkfun_autokit_driver_node:main',
            'mqtt_fleet_logger = rvr_autokit_driver.mqtt_fleet_logger_node:main',
        ],
    },
)

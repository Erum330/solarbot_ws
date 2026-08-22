from setuptools import find_packages, setup

package_name = 'solarbot_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erum_ifti',
    maintainer_email='erum8000@gmail.com',
    description=(
        'Bridges mros_interfaces (ESP32/panelbot2_ws) topics to the '
        'standard ROS 2 messages solarbot_ws expects, and back.'
    ),
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_bridge = solarbot_bridge.imu_bridge:main',
            'tof_bridge = solarbot_bridge.tof_bridge:main',
            'cmd_vel_bridge = solarbot_bridge.cmd_vel_bridge:main',
        ],
    },
)

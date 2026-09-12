import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'solarbot_safety'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files so ros2 launch can find them
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erum Iftikhar',
    maintainer_email='erum_ifti@todo.todo',
    description='Perimeter tracking and safety array nodes for Solarbot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'solarbot_perimeter_node = solarbot_safety.solarbot_perimeter_node:main',
            'solarbot_perimeter_openloop_node = solarbot_safety.solarbot_perimeter_openloop_node:main',
            'test_mtof_edge_detector = solarbot_safety.test_mtof_edge_detector:main',
            'test_straight_tof_stop = solarbot_safety.test_straight_tof_stop:main',
            'test_imu_turn = solarbot_safety.test_imu_turn:main',
            'test_timed_backup = solarbot_safety.test_timed_backup:main',
            'mtof_edge_follower = solarbot_safety.mtof_edge_follower:main',
            'view_mtof_gui = solarbot_safety.view_mtof_gui:main',
            'solarbot_perimeter_mtof_node = solarbot_safety.solarbot_perimeter_mtof_node:main',
            'solarbot_row_perimeter_node = solarbot_safety.solarbot_row_perimeter_node:main',
            'edge_logger_node = solarbot_safety.edge_logger_node:main',

        

            
        ],
    },
)
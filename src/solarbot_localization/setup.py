import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'solarbot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Erum Iftikhar',
    maintainer_email='erum8000@gmail.com',
    description='SolarBot camera-based (ECC) visual odometry, mapping, and ground-truth validation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transform_detector = solarbot_localization.transform_detector:main',
            'odom_mapper = solarbot_localization.odom_mapper:main',
            'real_odo = solarbot_localization.real_odo:main',
            'odom_logger = solarbot_localization.odom_logger:main',
        ],
    },
)

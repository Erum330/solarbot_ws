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
            'solarbot_roof_coverage_node = solarbot_safety.solarbot_roof_coverage_node:main',
        ],
    },
)
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot3_waypoint_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aadi',
    maintainer_email='aadi@todo.todo',
    description='TurtleBot3 waypoint navigation in ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_nav = turtlebot3_waypoint_nav.waypoint_nav:main',
        ],
    },
)

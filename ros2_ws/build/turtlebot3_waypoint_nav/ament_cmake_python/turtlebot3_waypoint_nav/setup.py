from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot3_waypoint_nav',
    version='0.0.0',
    packages=find_packages(
        include=('turtlebot3_waypoint_nav', 'turtlebot3_waypoint_nav.*')),
)

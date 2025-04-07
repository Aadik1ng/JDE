cmake_minimum_required(VERSION 3.5)
project(turtlebot3_waypoint_nav)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(turtlebot3_waypoint_nav)
ament_package()

## ROS 2 and TurtleBot 4 Navigation Setup

### Part 1 - ROS 2 Node Creation and TurtleBot 4 Simulation

#### Part 1 - a: Publisher/Subscriber Nodes

1.  **Workspace Setup:** `mkdir ros2_ws; cd ros2_ws; mkdir src; colcon build`
2.  **Package Creation:** `cd src/; ros2 pkg create publisher_subscriber_nodes --build-type ament_python --dependencies rclpy`
3.  **Node Development:** Create `publisher.py` and `subscriber.py` in `ros2_ws/src/publisher_subscriber_nodes/publisher_subscriber_nodes/`, update `setup.py` with `"publisher = publisher_subscriber_nodes.publisher:main", "subscriber = publisher_subscriber_nodes.subscriber:main"`, build: `cd ~/ros2_ws; colcon build; source install/setup.bash`
4.  **Node Execution:** `ros2 run publisher_subscriber_nodes publisher` (terminal 1), `ros2 run publisher_subscriber_nodes subscriber` (terminal 2).

#### Part 1 - b: TurtleBot 4 Simulation

1.  **Simulator Install:** `sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes`
2.  **Dev Tools Install:** `sudo apt install ros-dev-tools`
3.  **Ignition Fortress Install:** `sudo apt-get update && sudo apt-get install wget; sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb_release -cs\` main" > /etc/apt/sources.list.d/gazebo-stable.list'; wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -; sudo apt-get update && sudo apt-get install ignition-fortress`
4.  **SLAM/Nav2 Launch:** `ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true`

### Part 2 - TurtleBot 4 Navigation with Waypoints

#### Part 2 - a: Autonomous Navigation

1.  **Simulation Launch:** `ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true`
2.  **Waypoint Navigation:** Play simulation, undock TurtleBot, use Rviz "2D Pose Goal"/"Nav Through Poses" to define waypoints, "Start Nav Through Poses" to begin.

## References

* [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html)
* [Nav2 Installation](https://docs.nav2.org/getting_started/index.html#installation)
* [TurtleBot 4 Simulator Installation](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html) 

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = BasicNavigator()

    def navigate(self):
        self.navigator.waitUntilNav2Active()

        waypoints = [
            (1.0, 1.0, 0.0),  # First waypoint
            (2.0, 0.0, 0.0),  # Second waypoint
            (0.0, -1.0, 0.0)  # Third waypoint
        ]

        goal_poses = []
        for x, y, theta in waypoints:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)

        self.navigator.followWaypoints(goal_poses)

        while not self.navigator.isTaskComplete():
            self.get_logger().info("Navigating...")

        self.get_logger().info("Navigation complete!")

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    navigator.navigate()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


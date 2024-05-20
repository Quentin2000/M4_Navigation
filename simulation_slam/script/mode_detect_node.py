#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

class ModeCommandNode:
    def __init__(self):
        rospy.init_node('mode_detect_node')

        # Parameters
        self.cost_crawling = rospy.get_param("~cost_crawling", '45')  # Cost of mode 1
        self.cost_rolling = rospy.get_param("~cost_rolling", '30')   # Cost of mode 2

        # Subscribers
        rospy.Subscriber('/rtabmap/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/merged_occupancy_grid_inflated', OccupancyGrid, self.grid_callback)

        # Publisher
        self.mode_pub = rospy.Publisher('/m4assembly/mode/command', String, queue_size=10)

        # Variables
        self.current_goal = None
        self.current_grid = None

    def odom_callback(self, msg):

        # Check if grid and goal are available
        if self.current_grid is not None:

            # Extract robot position from odometry message
            x = int((msg.pose.pose.position.x - self.current_grid.info.origin.position.x) / self.current_grid.info.resolution)
            y = int((msg.pose.pose.position.y - self.current_grid.info.origin.position.y) / self.current_grid.info.resolution)

            # Get value from grid at robot position
            value = self.current_grid.data[y * self.current_grid.info.width + x]

            # Publish mode based on grid value
            if value == self.cost_rolling:          # ROLLING mode
                rospy.loginfo("Mode Rolling...")
                self.send_command("ROLLING")
            elif value == self.cost_crawling:       # CRAWLING mode
                rospy.loginfo("Mode Crawling...")
                self.send_command("CRAWLING")

    def goal_callback(self, msg):
        # Save current goal
        self.current_goal = msg

    def grid_callback(self, msg):
        # Save current grid
        self.current_grid = msg

    def send_command(self, value):
        # Publish value to topic
        self.mode_pub.publish(value)

if __name__ == '__main__':
    try:
        node = ModeCommandNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
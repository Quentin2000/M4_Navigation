#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

class GridMapProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('empty__grpid_pub', anonymous=True)
        
        # Publisher for the empty grid map
        self.grid_pub = rospy.Publisher('empty_grid', OccupancyGrid, queue_size=10)
        
        # Subscriber to the rtabmap grid map
        self.grid_sub = rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.grid_callback)

    def grid_callback(self, msg):
        # Create a new OccupancyGrid message
        empty_grid = OccupancyGrid()
        
        # Copy the header and info from the incoming message
        empty_grid.header = msg.header
        empty_grid.info = msg.info
        
        # Initialize the data with zeros (empty grid)
        empty_grid.data = [0] * len(msg.data)
        
        # Publish the empty grid
        self.grid_pub.publish(empty_grid)
        rospy.loginfo("Published an empty grid with the same header and info as the received grid.")

if __name__ == '__main__':
    try:
        grid_processor = GridMapProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from simulation_slam.msg import DualOccupancyGrid

class DualPointCloudToMap:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('dual_pc_2_map_node')

        # Parameters
        self.grid_resolution = rospy.get_param("~grid_resolution", 0.1)  # Grid cell size [m/cell]
        self.grid_size_meters = rospy.get_param("~grid_size_meters", 10) # Full grid size [m]
        # Thresholds
        self.threshold_1 = rospy.get_param("~threshold_1", 0.3) # Point Cloud cut threhsold in Z [m]
        self.threshold_2 = rospy.get_param("~threshold_2", 0.5) # Point Cloud cut threhsold in Z [m]
        # NOTE: Add your additional thresholds for new modes...
        # Input point cloud
        self.input_cloud = rospy.get_param("~input_cloud", 'rtabmap/cloud_obstacles')  # Point cloud to subscribe to
        # Output point occupancy maps
        self.output_map = rospy.get_param("~output_map", '/dual_grid_map')  # Grid map to publish
        self.output_map1 = rospy.get_param("~output_map1", '/crawl_grid_map')  # Grid map to publish for debug
        self.output_map2 = rospy.get_param("~output_map2", '/roll_grid_map')  # Grid map to publish for debug
        # NOTE: Add your additional output map for new modes...

        # Subscribers
        rospy.Subscriber(self.input_cloud, PointCloud2, self.point_cloud_callback)

        # Publishers
        self.dual_grid_map_pub = rospy.Publisher(self.output_map, DualOccupancyGrid, queue_size=1)
        self.grid_map_pub1 = rospy.Publisher(self.output_map1, OccupancyGrid, queue_size=1)
        self.grid_map_pub2 = rospy.Publisher(self.output_map2, OccupancyGrid, queue_size=1)
        # NOTE: Add your additional grid_map_pub for new modes...

        # Variables
        # Calculate grid size in [cells]
        self.grid_size_x = int(self.grid_size_meters / self.grid_resolution)
        self.grid_size_y = int(self.grid_size_meters / self.grid_resolution)
        # Initialize grid maps
        self.grid_map_1 = np.full((self.grid_size_x, self.grid_size_y), 1)
        self.grid_map_2 = np.full((self.grid_size_x, self.grid_size_y), 1)
        # NOTE: Add your additional grid_map for new modes...

    # Callback function for point cloud subscriber
    def point_cloud_callback(self, cloud_msg):

        # Convert point cloud to numpy array
        points = np.array(list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)))

        # Project point cloud into 2D grid map
        for point in points:    
            # Convert point cloud coordinates to grid coordinates
            grid_x = int(np.floor(point[0] / self.grid_resolution)) + self.grid_size_x // 2
            grid_y = int(np.floor(point[1] / self.grid_resolution)) + self.grid_size_y // 2

            # Check if grid coordinates are within the grid map bounds
            if 0 <= grid_x < self.grid_size_x and 0 <= grid_y < self.grid_size_y:
                if point[2] < self.threshold_1:
                    # Mark grid cell as occupied for map 1
                    self.grid_map_1[grid_y, grid_x] = 100
                if point[2] < self.threshold_2:
                    # Mark grid cell as occupied for map 2
                    self.grid_map_2[grid_y, grid_x] = 100
                # NOTE: Add your additional check for new modes...

        # Publish grid map
        self.publish_grid_map()

    def initialize_occupancy_grid_msg(self, grid_map, time):
        # Create OccupancyGrid message
        grid_msg = OccupancyGrid()

        # Initialize OccupancyGrid messages with grid_map data
        grid_msg.header.stamp = time
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.grid_resolution
        grid_msg.info.width = self.grid_size_x
        grid_msg.info.height = self.grid_size_y
        grid_msg.info.origin.position.x = - (self.grid_size_x // 2) * self.grid_resolution
        grid_msg.info.origin.position.y = - (self.grid_size_y // 2) * self.grid_resolution
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = grid_map.flatten().tolist()    # Flatten grid map and set as occupancy data

        return grid_msg


    def publish_grid_map(self):

        time = rospy.Time.now()
        
        # Publish grid 1
        grid_msg1 = self.initialize_occupancy_grid_msg(self.grid_map_1, time)
        self.grid_map_pub1.publish(grid_msg1)
        # Publish grid 2
        grid_msg2 = self.initialize_occupancy_grid_msg(self.grid_map_2, time)
        self.grid_map_pub2.publish(grid_msg2)
        # Publish grid N
        # NOTE: Add your additional grid_msgN initialization for new modes...

        # Create DualOccupancyGrid message
        dual_grid_msg = DualOccupancyGrid()
        dual_grid_msg.grid1 = grid_msg1
        dual_grid_msg.grid2 = grid_msg2
        # NOTE: Modify DualOccupancyGrid() and add your additional grid_msgN for new modes...

        # Publish dual grid map
        self.dual_grid_map_pub.publish(dual_grid_msg)

if __name__ == '__main__':
    try:
        node = DualPointCloudToMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
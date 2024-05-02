#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid

# Define global variables
grid_map_pub = None

# Callback function for point cloud subscriber
def point_cloud_callback(cloud_msg):
    global grid_map
    
    # Convert point cloud to numpy array
    # See doucmentation on PointCloud2 message parameters (field names): https://wiki.ros.org/pcl/Overview#Common_PointCloud2_field_names
    points = np.array(list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)))

    # print("points :", points)

    # Project point cloud into 2D grid map
    for point in points:
        # Transform 3D point to grid coordinates
        grid_x = int(np.floor(point[0] / grid_resolution)) + grid_size_x // 2
        grid_y = int(np.floor(point[1] / grid_resolution)) + grid_size_y // 2

        # Check if grid coordinates are within the grid map bounds
        if 0 <= grid_x < grid_size_x and 0 <= grid_y < grid_size_y:
            # Mark grid cell as occupied
            grid_map[grid_y, grid_x] = 100

    rospy.loginfo("Grid Resolution: %s", grid_resolution)

    # Publish grid map as OccupancyGrid message
    publish_grid_map()

def publish_grid_map():

    # Create OccupancyGrid message
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.Time.now()
    grid_msg.header.frame_id = "map"
    grid_msg.info.resolution = grid_resolution
    grid_msg.info.width = grid_size_x
    grid_msg.info.height = grid_size_y
    grid_msg.info.origin.position.x = - (grid_size_x // 2) * grid_resolution
    grid_msg.info.origin.position.y = - (grid_size_y // 2) * grid_resolution
    grid_msg.info.origin.position.z = 0.0
    grid_msg.info.origin.orientation.x = 0.0
    grid_msg.info.origin.orientation.y = 0.0
    grid_msg.info.origin.orientation.z = 0.0
    grid_msg.info.origin.orientation.w = 1.0

    # Flatten grid map and set as occupancy data
    grid_msg.data = grid_map.flatten().tolist()

    # Publish grid map
    grid_map_pub.publish(grid_msg)

def main():
    global grid_map
    global grid_map_pub
    global grid_resolution
    global grid_size_meters
    global grid_size_x
    global grid_size_y

    # Initialize ROS node
    rospy.init_node('pointCloud2GridMap_node')

    grid_resolution = rospy.get_param("~grid_resolution", 0.1)  # Grid cell size [m/cell]
    grid_size_meters = rospy.get_param("~grid_size_meters", 10) # Full grid size [m]
    # mode_cost = rospy.get_param("~mode_cost", 1) # Grid cost for free cells between 1 and 100 (depends on mode FLYING, CRAWLING, or ROLLING) 
    input_cloud = rospy.get_param("~input_cloud", 'crawl_detect_cloud')  # Point cloud to subscribe to
    output_map = rospy.get_param("~output_map", '/crawl_grid_map')  # Grid map to publish
    grid_size_x = int(grid_size_meters / grid_resolution) # Number of grid cells in X direction [cells]
    grid_size_y = int(grid_size_meters / grid_resolution) # Number of grid cells in Y direction [cells]
    grid_map = np.full((grid_size_x, grid_size_y), 1)

    # Subscribe to point cloud topic
    rospy.Subscriber(input_cloud, PointCloud2, point_cloud_callback)

    # Advertise grid map topic
    grid_map_pub = rospy.Publisher(output_map, OccupancyGrid, queue_size=1)

    # Spin ROS node
    rospy.spin()

if __name__ == '__main__':
    main()

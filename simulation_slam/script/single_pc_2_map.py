#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid

class PointCloudToGridMap:
    def __init__(self):

        rospy.init_node('single_pc_2_map_node')
        
        # Parameters   
        self.grid_resolution = rospy.get_param("~grid_resolution", 0.1)
        self.grid_size_meters = rospy.get_param("~grid_size_meters", 10)
        input_cloud = rospy.get_param("~input_cloud", 'crawl_detect_cloud')
        output_map = rospy.get_param("~output_map", '/crawl_grid_map')

        # Subscriber
        rospy.Subscriber(input_cloud, PointCloud2, self.point_cloud_callback)
        
        # Publisher
        self.grid_map_pub = rospy.Publisher(output_map, OccupancyGrid, queue_size=1)

        # Variables
        self.grid_size_x = int(self.grid_size_meters / self.grid_resolution)
        self.grid_size_y = int(self.grid_size_meters / self.grid_resolution)
        self.grid_map = np.full((self.grid_size_x, self.grid_size_y), 1)


    def point_cloud_callback(self, cloud_msg):


        points = np.array(list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)))
        for point in points:
            # Convert point cloud coordinates to grid coordinates
            grid_x = int(np.floor(point[0] / self.grid_resolution)) + self.grid_size_x // 2
            grid_y = int(np.floor(point[1] / self.grid_resolution)) + self.grid_size_y // 2
            # Check if grid coordinates are within the grid map bounds
            if 0 <= grid_x < self.grid_size_x and 0 <= grid_y < self.grid_size_y:
                # Mark grid cell as occupied
                self.grid_map[grid_y, grid_x] = 100
        self.publish_grid_map()

    def publish_grid_map(self):

        # Create grid_msg
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
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
        grid_msg.data = self.grid_map.flatten().tolist()
        
        # Publish grid_msg
        self.grid_map_pub.publish(grid_msg)



if __name__ == '__main__':

    try:
        node = PointCloudToGridMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in main function: %s", str(e))

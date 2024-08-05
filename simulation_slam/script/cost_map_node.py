#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from simulation_slam.msg import DualOccupancyGrid
from simulation_slam.msg import OccupancyGridFlagged
import numpy as np
from scipy.ndimage import binary_dilation 

class OccupancyGridMerger:
    def __init__(self):

        rospy.init_node('occupancy_grid_merger', anonymous=True)

        # PARAMETERS
        # Input grids
        input_grid_rolling = rospy.get_param("~input_grid_rolling", '/roll_grid_map')     
        input_grid_crawling = rospy.get_param("~input_grid_crawling", '/crawl_grid_map')    
        # NOTE: Add your additional input_grids for new modes...
        # Dual input grid
        self.input_grid_dual = rospy.get_param("~input_grid_dual", 'None')     
        # NOTE: Modify input_grid_dual to input_grid_N for new modes...
        # Cost within [0:100] to traverse the grid
        self.cost_rolling = rospy.get_param("~cost_rolling", 30) 
        self.cost_crawling = rospy.get_param("~cost_crawling", 45)
        # NOTE: Add your additional costs for new modes...
        # Ouput grids
        self.output_grid = rospy.get_param("~output_grid", '/merged_occupancy_grid')  # Grid map to publish
        self.merged_occupancy_grid_pub_inflated_topic = self.output_grid + "_inflated"

        # SUBSCRIBERS
        if self.input_grid_dual == 'None':
            # Subscribe to the occupancy grid topics
            rospy.Subscriber(input_grid_rolling, OccupancyGrid, self.occupancy_grid_rolling_callback)
            rospy.Subscriber(input_grid_crawling, OccupancyGrid, self.occupancy_grid_crawling_callback)
            # NOTE: Add your additional subscribers for new modes...
        else:
            rospy.Subscriber(self.input_grid_dual, DualOccupancyGrid, self.occupancy_grid_dual_callback)
            # NOTE: Modify input_grid_dual to input_grid_N for new modes...

        # PUBLISHERS
        self.merged_occupancy_grid_pub = rospy.Publisher(self.output_grid, OccupancyGrid, queue_size=10)
        self.merged_occupancy_grid_pub_inflated = rospy.Publisher(self.merged_occupancy_grid_pub_inflated_topic, OccupancyGrid, queue_size=10)

        # VARIABLES
        self.occupancy_grid_crawling = None
        self.occupancy_grid_rolling = None
        # Set the rate at which the merging operation is performed (2Hz)
        self.rate = rospy.Rate(2)

    def occupancy_grid_dual_callback(self, msg):
        # Store the received occupancy grid data
        self.occupancy_grid_crawling = msg.grid1
        self.occupancy_grid_rolling = msg.grid2
        # Call the function to merge the occupancy grids
        self.merge_occupancy_grids()

    def occupancy_grid_crawling_callback(self, msg):
        # Store the received occupancy grid data
        self.occupancy_grid_crawling = msg
        # Call the function to merge the occupancy grids
        self.merge_occupancy_grids()

    def occupancy_grid_rolling_callback(self, msg):
        # Store the received occupancy grid data
        self.occupancy_grid_rolling = msg
        # Call the function to merge the occupancy grids
        self.merge_occupancy_grids()

    def merge_occupancy_grids(self):
        # Check if both occupancy grids are received
        if self.occupancy_grid_crawling is not None and self.occupancy_grid_rolling is not None:

            # Merge occupancy grids
            occupancy_grid_crawling = np.array(self.occupancy_grid_crawling.data, dtype=np.int32)
            occupancy_grid_rolling = np.array(self.occupancy_grid_rolling.data, dtype=np.int32) 
            # NOTE: Add your additional occupancy_grid_N
            merged_data = np.minimum(occupancy_grid_crawling * self.cost_crawling, occupancy_grid_rolling * self.cost_rolling)
            # NOTE: Add new merged data: merged_data = np.minimum(occupancy_grid_N * self.cost_N, merged_data)

            # Set maximum value as 100 (obstacles)
            occupancy_grid_merged_data = np.minimum(merged_data, 100).astype(np.int8)

            # Create a new occupancy grid message
            merged_occupancy_grid = OccupancyGrid()
            merged_occupancy_grid.header.stamp = rospy.Time.now()
            merged_occupancy_grid.header.frame_id = "map"
            merged_occupancy_grid.info = self.occupancy_grid_crawling.info
            merged_occupancy_grid.data = occupancy_grid_merged_data

            # Publish the merged occupancy grid message
            self.merged_occupancy_grid_pub.publish(merged_occupancy_grid)

            # Inflate the costs from higher one to lower one to make sure that the robot is always in the most optimal configuration before entering a different mode zone. 
            occupancy_grid_merged_data = self.inflate_cost(occupancy_grid_merged_data.reshape((self.occupancy_grid_crawling.info.width, self.occupancy_grid_crawling.info.height)), 100, 1)
            occupancy_grid_merged_data = self.inflate_cost(occupancy_grid_merged_data.reshape((self.occupancy_grid_crawling.info.width, self.occupancy_grid_crawling.info.height)), self.cost_crawling, 10)
            # NOTE: Add your additional inflation based on cost of mode, starting from highest to lowest cost

            # Create a new occupancy grid to match the size of the larger occupancy grid
            merged_occupancy_grid_inflated = OccupancyGrid()
            merged_occupancy_grid_inflated.header.stamp = rospy.Time.now()
            merged_occupancy_grid_inflated.header.frame_id = "map"
            merged_occupancy_grid_inflated.info = self.occupancy_grid_crawling.info
            merged_occupancy_grid_inflated.data = occupancy_grid_merged_data
            
            # Publish the merged occupancy grid
            self.merged_occupancy_grid_pub_inflated.publish(merged_occupancy_grid_inflated)

            # Sleep to control the rate
            self.rate.sleep()

    def inflate_cost(self, grid, cost, radius):
        # Create mask where grid cell value = cost
        cost_mask = grid == cost
        # Create obstacle mask where grid cell value = 100
        obstacle_mask = grid == 100
        # Create inflation pattern as circle
        selem = np.ones((2*radius + 1, 2*radius +1))
        # Create inflation mask where grid cell value = cost
        inflated_cost_mask = binary_dilation(cost_mask, structure=selem)
        # Apply inflation mask to grid 
        inflated_grid = np.where(inflated_cost_mask, cost, grid)
        # Regenerate and inflate obstacles
        inflated_grid_2 = np.where(obstacle_mask, 100, inflated_grid)

        return inflated_grid_2.flatten()

if __name__ == '__main__':
    try:
        occupancy_grid_merger = OccupancyGridMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from simulation_slam.msg import DualOccupancyGrid
import numpy as np
from scipy.ndimage import binary_dilation 

class OccupancyGridMerger:
    def __init__(self):

        rospy.init_node('occupancy_grid_merger', anonymous=True)

        input_grid1 = rospy.get_param("~input_grid1", '/crawling_grid_map')  # Grid map to subscribe to
        input_grid2 = rospy.get_param("~input_grid2", '/rolling_grid_map')  # Grid map to subscribe to
        self.input_grid_dual = rospy.get_param("~input_grid_dual", 'None')  # Grid map to subscribe to

        self.grid_cost1 = rospy.get_param("~grid_cost1", 40) # Cost within [0:100] to traverse the grid
        self.grid_cost2 = rospy.get_param("~grid_cost2", 20) # Cost within [0:100] to traverse the grid

        self.output_grid = rospy.get_param("~output_grid", '/merged_occupancy_grid')  # Grid map to publish
        self.merged_occupancy_grid_pub_inflated_topic = self.output_grid + "_inflated"

        if self.input_grid_dual == 'None':
            # Subscribe to the occupancy grid topics
            rospy.Subscriber(input_grid1, OccupancyGrid, self.occupancy_grid1_callback)
            rospy.Subscriber(input_grid2, OccupancyGrid, self.occupancy_grid2_callback)
        else:
            rospy.Subscriber(self.input_grid_dual, DualOccupancyGrid, self.occupancy_grid_dual_callback)

        # Publisher to publish the merged occupancy grid
        self.merged_occupancy_grid_pub = rospy.Publisher(self.output_grid, OccupancyGrid, queue_size=10)
        self.merged_occupancy_grid_pub_inflated = rospy.Publisher(self.merged_occupancy_grid_pub_inflated_topic, OccupancyGrid, queue_size=10)

        # Initialize variables to hold the occupancy grid data
        self.occupancy_grid1 = None
        self.occupancy_grid2 = None

        # Set the rate at which the merging operation is performed (2Hz)
        self.rate = rospy.Rate(2)

    def occupancy_grid_dual_callback(self, msg):
        # Store the received occupancy grid data
        self.occupancy_grid1 = msg.grid1
        self.occupancy_grid2 = msg.grid2
        # Call the function to merge the occupancy grids
        self.merge_occupancy_grids()

    def occupancy_grid1_callback(self, msg):
        # Store the received occupancy grid data
        self.occupancy_grid1 = msg
        # Call the function to merge the occupancy grids
        self.merge_occupancy_grids()

    def occupancy_grid2_callback(self, msg):
        # Store the received occupancy grid data
        self.occupancy_grid2 = msg
        # Call the function to merge the occupancy grids
        self.merge_occupancy_grids()

    def merge_occupancy_grids(self):
        # Check if both occupancy grids are received
        if self.occupancy_grid1 is not None and self.occupancy_grid2 is not None:

            # occupancy_grid_array1 = np.minimum(np.array(self.occupancy_grid1.data)*self.grid_cost1, 100)
            # occupancy_grid_array2 = np.minimum(np.array(self.occupancy_grid2.data)*self.grid_cost2, 100)
            # occupancy_grid_merged_data = np.minimum(occupancy_grid_array1, occupancy_grid_array2).astype(np.int8)

            occupancy_grid_data1 = np.array(self.occupancy_grid1.data, dtype=np.int32)
            occupancy_grid_data2 = np.array(self.occupancy_grid2.data, dtype=np.int32) 
            merged_data = np.minimum(occupancy_grid_data1 * self.grid_cost1, occupancy_grid_data2 * self.grid_cost2)
            occupancy_grid_merged_data = np.minimum(merged_data, 100).astype(np.int8)

            # Create a new occupancy grid to match the size of the larger occupancy grid
            merged_occupancy_grid = OccupancyGrid()
            merged_occupancy_grid.header.stamp = rospy.Time.now()
            merged_occupancy_grid.header.frame_id = "map"
            merged_occupancy_grid.info = self.occupancy_grid1.info
            merged_occupancy_grid.data = occupancy_grid_merged_data

            # Publish the merged occupancy grid
            self.merged_occupancy_grid_pub.publish(merged_occupancy_grid)

            # Inflate the costs from higher one to lower one to make sure that the robot is always in the most optimal configuration before entering a different mode zone. 
            # occupancy_grid_merged_data = self.increase_coverage(occupancy_grid_merged_data, self.occupancy_grid1.info.width, self.occupancy_grid1.info.height, 40, 10)
            # if self.input_grid_dual == 'None':
            occupancy_grid_merged_data = self.inflate_cost(occupancy_grid_merged_data.reshape((self.occupancy_grid1.info.width, self.occupancy_grid1.info.height)), 100, 1)
            occupancy_grid_merged_data = self.inflate_cost(occupancy_grid_merged_data.reshape((self.occupancy_grid1.info.width, self.occupancy_grid1.info.height)), 40, 10)

            # Create a new occupancy grid to match the size of the larger occupancy grid
            merged_occupancy_grid_inflated = OccupancyGrid()
            merged_occupancy_grid_inflated.header.stamp = rospy.Time.now()
            merged_occupancy_grid_inflated.header.frame_id = "map"
            merged_occupancy_grid_inflated.info = self.occupancy_grid1.info
            merged_occupancy_grid_inflated.data = occupancy_grid_merged_data

            # Publish the merged occupancy grid
            self.merged_occupancy_grid_pub_inflated.publish(merged_occupancy_grid_inflated)

            # Sleep to control the rate
            self.rate.sleep()

    def inflate_cost(self, grid, cost, radius):
        cost_mask = grid == cost
        obstacle_mask = grid == 100 # Obstacle cost

        selem = np.ones((2*radius + 1, 2*radius +1))
        inflated_cost_mask = binary_dilation(cost_mask, structure=selem)

        inflated_grid = np.where(inflated_cost_mask, cost, grid)
        inflated_grid_2 = np.where(obstacle_mask, 100, inflated_grid)

        return inflated_grid_2.flatten()

if __name__ == '__main__':
    try:
        occupancy_grid_merger = OccupancyGridMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from simulation_slam.msg import DualOccupancyGrid
from simulation_slam.msg import OccupancyGridFlagged
import numpy as np
from scipy.ndimage import binary_dilation 

class GridInflater:
    def __init__(self):

        rospy.init_node('inflate_grid', anonymous=True)

        # PARAMETERS
        # Input grids
        input_grid = rospy.get_param("~input_grid", '/nav_grid_map')     
   
        # Ouput grids
        self.output_grid = rospy.get_param("~output_grid", '/nav_grid_map_inflated')

        # SUBSCRIBERS
        rospy.Subscriber(input_grid, OccupancyGrid, self.grid_callback)

        # PUBLISHERS
        self.output_grid_pub = rospy.Publisher(self.output_grid, OccupancyGrid, queue_size=10)

        # VARIABLES
        self.grid = None

        # Set the rate at which the merging operation is performed (5Hz)
        self.rate = rospy.Rate(5)

    def grid_callback(self, msg):
        # Store the received occupancy grid data
        self.grid = msg
        # Call the function to merge the occupancy grids
        self.inflate_grid()

    def inflate_grid(self):
        # Check if both occupancy grids are received
        if self.grid is not None:

            width = self.grid.info.width
            height = self.grid.info.height
            grid_array = np.array(self.grid.data).reshape((height, width))

            # Inflate the costs from higher one to lower one to make sure that the robot is always in the most optimal configuration before entering a different mode zone. 
            inflated_grid_data = self.inflate_cost(grid_array, 100, 1)

            # Convert the 2D array back to a 1D list
            inflated_grid_list = list(inflated_grid_data.flatten())

            # Create a new occupancy grid to match the size of the larger occupancy grid
            inflated_grid = OccupancyGrid()
            inflated_grid.header.stamp = rospy.Time.now()
            inflated_grid.header.frame_id = "map"
            inflated_grid.info = self.grid.info
            inflated_grid.data = inflated_grid_list
            
            # Publish the merged occupancy grid
            self.output_grid_pub.publish(inflated_grid)

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
        inflated_grid = np.where(obstacle_mask, 100, inflated_grid)

        return inflated_grid

if __name__ == '__main__':
    try:
        grid_inflater = GridInflater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
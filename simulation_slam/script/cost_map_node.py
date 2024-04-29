#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

class OccupancyGridMerger:
    def __init__(self):

        rospy.init_node('occupancy_grid_merger', anonymous=True)

        input_grid1 = rospy.get_param("~input_grid1", '/crawling_grid_map')  # Grid map to subscribe to
        input_grid2 = rospy.get_param("~input_grid2", '/rolling_grid_map')  # Grid map to subscribe to

        self.grid_cost1 = rospy.get_param("~grid_cost1", 1.0) # Cost within [0:1] to traverse the grid
        self.grid_cost2 = rospy.get_param("~grid_cost2", 0.5) # Cost within [0:1] to traverse the grid

        self.output_grid = rospy.get_param("~output_grid", '/merged_occupancy_grid')  # Grid map to publish

        # Subscribe to the first occupancy grid topic
        rospy.Subscriber(input_grid1, OccupancyGrid, self.occupancy_grid1_callback)

        # Subscribe to the second occupancy grid topic
        rospy.Subscriber(input_grid2, OccupancyGrid, self.occupancy_grid2_callback)

        # Publisher to publish the merged occupancy grid
        self.merged_occupancy_grid_pub = rospy.Publisher(self.output_grid, OccupancyGrid, queue_size=10)

        # Initialize variables to hold the occupancy grid data
        self.occupancy_grid1 = None
        self.occupancy_grid2 = None

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

            # width2 = self.occupancy_grid2.info.width
            # height2 = self.occupancy_grid2.info.height
            # occupancy_grid2_array = np.array(self.occupancy_grid2.data).reshape(width2, height2)

            # # Calculate the starting and ending indices for rows and columns
            # center_row = height2 // 2
            # center_col = width2 // 2
            # half_rows = self.occupancy_grid2.info.height // 2
            # half_cols = self.occupancy_grid2.info.width // 2
            # start_row = center_row - half_rows
            # end_row = center_row + half_rows
            # start_col = center_col - half_cols
            # end_col = center_col + half_cols

            # # Extract the desired region from the map array
            # occupancy_grid2_array_small = occupancy_grid2_array[start_row:end_row, start_col:end_col]

            # # Flatten the smaller map array
            # flattened_occupancy_grid2_array_small = occupancy_grid2_array_small.flatten()

            occupancy_grid_array1 = np.array(self.occupancy_grid1.data)
            occupancy_grid_array2 = np.array(self.occupancy_grid2.data)
            occupancy_grid_merged_data = np.minimum(self.grid_cost1*occupancy_grid_array1 + self.grid_cost2*occupancy_grid_array2, 100).astype(np.int8)

            # Create a new occupancy grid to match the size of the larger occupancy grid
            merged_occupancy_grid = OccupancyGrid()
            merged_occupancy_grid.header.stamp = rospy.Time.now()
            merged_occupancy_grid.header.frame_id = "map"
            merged_occupancy_grid.info = self.occupancy_grid1.info
            merged_occupancy_grid.data = occupancy_grid_merged_data

            # Publish the merged occupancy grid
            self.merged_occupancy_grid_pub.publish(merged_occupancy_grid)

if __name__ == '__main__':
    try:
        occupancy_grid_merger = OccupancyGridMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# import rospy
# from nav_msgs.msg import OccupancyGrid
# import numpy as np

# def occupancy_grid1_callback(msg):
#     global occupancy_grid1
#     # Store the received occupancy grid data
#     occupancy_grid1 = msg
#     # Call the function to merge the occupancy grids
#     merge_occupancy_grids()

# def occupancy_grid2_callback(msg):
#     global occupancy_grid2
#     # Store the received occupancy grid data
#     occupancy_grid2 = msg
#     # Call the function to merge the occupancy grids
#     merge_occupancy_grids()

# def merge_occupancy_grids():
#     # Check if both occupancy grids are received
#     if occupancy_grid1 is not None and occupancy_grid2 is not None:
#         # Convert occupancy grid data to NumPy arrays for efficient computation
#         grid1 = np.array(occupancy_grid1.data).reshape(occupancy_grid1.info.height, occupancy_grid1.info.width)
#         grid2 = np.array(occupancy_grid2.data).reshape(occupancy_grid2.info.height, occupancy_grid2.info.width)

#         # Get dimensions of both occupancy grids
#         rows1, cols1 = grid1.shape
#         rows2, cols2 = grid2.shape

#         # Determine the difference in size between the two occupancy grids
#         row_diff = rows2 - rows1
#         col_diff = cols2 - cols1

#         # Create a new occupancy grid to match the size of the larger occupancy grid
#         merged_occupancy_grid = OccupancyGrid()
#         merged_occupancy_grid.header.stamp = rospy.Time.now()
#         merged_occupancy_grid.info = occupancy_grid1.info

#         # Expand the smaller occupancy grid by adding zero cells around its edges
#         if row_diff > 0:
#             zero_rows = np.zeros((row_diff, cols1), dtype=np.int8)
#             grid1 = np.vstack((grid1, zero_rows))
#             merged_occupancy_grid.info.height += row_diff
#             merged_occupancy_grid.info.origin.position.y = - (merged_occupancy_grid.info.height // 2) * merged_occupancy_grid.info.resolution
#         elif row_diff < 0:
#             zero_rows = np.zeros((-row_diff, cols2), dtype=np.int8)
#             grid2 = np.vstack((grid2, zero_rows))
#         if col_diff > 0:
#             zero_cols = np.zeros((rows1, col_diff), dtype=np.int8)
#             grid1 = np.hstack((grid1, zero_cols))
#             merged_occupancy_grid.info.width += col_diff
#             merged_occupancy_grid.info.origin.position.x = - (merged_occupancy_grid.info.width // 2) * merged_occupancy_grid.info.resolution
#         elif col_diff < 0:
#             zero_cols = np.zeros((rows2, -col_diff), dtype=np.int8)
#             grid2 = np.hstack((grid2, zero_cols))

#         # Sum the values of the corresponding cells in the two occupancy grids
#         merged_grid = min(0.5*grid1 + grid2, 100)

#         # Convert the merged grid back to a 1D list
#         merged_occupancy_grid.data = merged_grid.flatten().tolist()

#         # Publish the merged occupancy grid
#         merged_occupancy_grid_pub.publish(merged_occupancy_grid)

# def main():
#     global merged_occupancy_grid_pub

#     rospy.init_node('cost_map_node', anonymous=True)

#     # Subscribe to the first occupancy grid topic
#     rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, occupancy_grid1_callback)

#     # Subscribe to the second occupancy grid topic
#     rospy.Subscriber('/crawl_grid_map', OccupancyGrid, occupancy_grid2_callback)

#     # Publisher to publish the merged occupancy grid
#     merged_occupancy_grid_pub = rospy.Publisher('/merged_occupancy_grid', OccupancyGrid, queue_size=10)

#     rospy.spin()

# if __name__ == '__main__':
#     main()
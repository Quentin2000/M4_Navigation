from scipy.ndimage import binary_dilation 
import numpy as np

input = [100,   1,   1,   1,   1,   1, 100,   1,   1,   1,   1, 100, 100,   1, 100, 100,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   40,   1,   1,   1,   1,   1,   40,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1]

input2 = [100,   100,   100,   1,   1,   1, 100,   1,   1,   1,   1, 100, 100,   1, 100, 100,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   100,   1,   1,   100,   1,   1,   1,   1,   1,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
   1,   1,   1,   1,   1,   1,   1,   1,   1,   1]

print(len(input))

input_np = np.array(input)


def inflate_cost(grid, cost, radius):
   cost_mask = grid == cost
   obstacle_mask = grid == 100 # Obstacle cost

   selem = np.ones((2*radius + 1, 2*radius +1))
   inflated_cost_mask = binary_dilation(cost_mask, structure=selem)

   print(inflated_cost_mask)

   inflated_grid = np.where(inflated_cost_mask, cost, grid)

   print(inflated_grid)

   inflated_grid = np.where(obstacle_mask, 100, grid)

   print(inflated_grid.flatten())

inflate_cost(input_np.reshape((25, 4)), 40, 1)


import rospy
from sensor_msgs.msg import PointCloud2
import pcl
import numpy as np
import cv2

def pointcloud_callback(msg):
    # Convert PointCloud2 message to PCL point cloud
    cloud = pcl.PointCloud()
    points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
    cloud.from_array(points.astype(np.float32))

    # Filter points based on altitude
    altitude_threshold = 2.0  # Example threshold
    filtered_cloud = cloud.extract([f"z > {altitude_threshold}", ""])

    # Generate grid map
    resolution = 0.1  # Example resolution
    min_x, max_x = -10, 10  # Example x-axis range
    min_y, max_y = -10, 10  # Example y-axis range
    grid_map = np.zeros(((max_y - min_y) / resolution, (max_x - min_x) / resolution), dtype=np.uint8)

    for point in filtered_cloud:
        x_idx = int((point[0] - min_x) / resolution)
        y_idx = int((point[1] - min_y) / resolution)
        if 0 <= x_idx < grid_map.shape[1] and 0 <= y_idx < grid_map.shape[0]:
            grid_map[y_idx, x_idx] = 255  # Set obstacle cell to white

    # Visualize grid map (for debugging)
    cv2.imshow("Grid Map", grid_map)
    cv2.waitKey(1)

def main():
    rospy.init_node('custom_gridmap_generator')
    rospy.Subscriber('/camera/pointcloud', PointCloud2, pointcloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

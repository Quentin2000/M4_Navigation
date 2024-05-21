import sys
import os
import bagpy
import pandas as pd

def save_rosbag_to_excel(bag_file, output_file):

    # Construct the full path to the bag file
    bag_directory = os.path.join("..", "bagfiles")  # Assuming the bag files are in the folder "bagfiles" one level above
    bag_file_path = os.path.join(bag_directory, bag_file)
    
    # Load the bag file
    bag = bagpy.bagreader(bag_file_path)

    # Read all messages from the bag file
    for topic in bag.topics:
        print(f"Topic: '{topic}'")
        # Read messages from the current topic
        csv_file = bag.message_by_topic(topic)

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # Use default bag file name
        default_bag_file = "speed_norm_idle.bag"
        default_output_file = "speed_norm_idle.xlsx"
        save_rosbag_to_excel(default_bag_file, default_output_file)
    elif len(sys.argv) == 2:
        bag_file = sys.argv[1]
        default_output_file = "rosbag_data.xlsx"
        save_rosbag_to_excel(bag_file, default_output_file)
    elif len(sys.argv) == 3:
        bag_file = sys.argv[1]
        output_file = sys.argv[2]
        save_rosbag_to_excel(bag_file, output_file)
    else:
        print("Usage: python save_rosbag_to_excel.py [<bag_file>] [<output_file>]")

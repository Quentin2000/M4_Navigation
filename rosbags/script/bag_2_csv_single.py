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

    # Initialize a DataFrame to store merged data
    merged_df = None

    # Read all messages from the bag file
    for topic in bag.topics:
        print(f"Topic: '{topic}'")
        # Read messages from the current topic
        csv_file = bag.message_by_topic(topic)

        # Read CSV file into a DataFrame
        df = pd.read_csv(csv_file)

        # Extract the Time column
        time_column = df['Time']

        # Drop the Time column from the DataFrame
        df = df.drop(columns=['Time'])

        # Prefix each column name with the topic name
        df.columns = [f"{topic}/{col}" for col in df.columns]

        # If merged_df is None, assign df to it
        if merged_df is None:
            merged_df = df
        else:
            # Merge the current DataFrame with merged_df based on index
            merged_df = pd.merge(merged_df, df, left_index=True, right_index=True)

    # Insert the Time column back into the merged DataFrame
    merged_df.insert(0, 'Time', time_column)

    # Write the merged DataFrame to a CSV file
    merged_df.to_csv(output_file, index=False)

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # Use default bag file name
        default_bag_file = "speed_norm_idle.bag"
        default_bag_file_without_extension = os.path.splitext(default_bag_file)[0]
        default_output_file = f"../bagfiles/{default_bag_file_without_extension}/{default_bag_file_without_extension}.csv"
        save_rosbag_to_excel(default_bag_file, default_output_file)
    elif len(sys.argv) == 2:
        bag_file = sys.argv[1]
        default_bag_file_without_extension = os.path.splitext(bag_file)[0]
        default_output_file = f"../bagfiles/{default_bag_file_without_extension}/{default_bag_file_without_extension}.csv"
        save_rosbag_to_excel(bag_file, default_output_file)
    elif len(sys.argv) == 3:
        bag_file = sys.argv[1]
        output_file = sys.argv[2]
        save_rosbag_to_excel(bag_file, output_file)
    else:
        print("Usage: python save_rosbag_to_excel.py [<bag_file>] [<output_file>]")

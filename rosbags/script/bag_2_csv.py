import sys
import os
import bagpy
import pandas as pd

def merge_rosbag_topics(bag_file, output_file):
    # Construct the full path to the bag file
    bag_directory = os.path.join("..", "bagfiles", os.path.splitext(bag_file)[0])  # Assuming the bag files are in the folder "bagfiles" one level above
    bag_file_path = os.path.join(bag_directory, bag_file)

    # Create a bag reader instance
    bag = bagpy.bagreader(bag_file_path)
    
    # Dictionary to store DataFrames for each topic
    topic_dfs = []

    # Process each topic in the bag file
    for topic in bag.topics:
        # Remove the leading slash if present
        formatted_topic = topic.lstrip('/')

        # Extract messages for the current topic and read into DataFrame
        csv_path = bag.message_by_topic(topic)
        df = pd.read_csv(csv_path)

        # Rename columns to include topic name, keep Time as is
        df.rename(columns={col: f"{formatted_topic}_{col}" if col != 'Time' else 'Time' for col in df.columns}, inplace=True)
        
        # Append the modified DataFrame to the list
        topic_dfs.append(df)

    # Initialize a DataFrame from the first topic
    all_data = topic_dfs[0]

    # Merge each topic's data into one DataFrame using outer join on 'Time'
    for df in topic_dfs[1:]:
        all_data = pd.merge(all_data, df, on='Time', how='outer')

    # Sort by Time
    all_data.sort_values('Time', inplace=True)
    
    # Fill missing data with the previous row's values and then fill remaining NaNs with 0
    all_data.fillna(method='ffill', inplace=True)  # Fill with previous data
    all_data.fillna(0, inplace=True)               # Fill remaining NaNs with 0

    # Save to CSV
    all_data.to_csv(output_file, index=False)
    print(f"Data merged and saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # Use default bag file name
        default_bag_file = "speed_norm_idle.bag"
        default_output_file = "../bagfiles/speed_norm_idle/speed_norm_idle.csv"
        merge_rosbag_topics(default_bag_file, default_output_file)
    elif len(sys.argv) == 2:
        bag_file = sys.argv[1]
        output_file = f"../bagfiles/{os.path.splitext(bag_file)[0]}/{os.path.splitext(bag_file)[0]}.csv"
        merge_rosbag_topics(bag_file, output_file)
    elif len(sys.argv) == 3:
        bag_file = sys.argv[1]
        output_file = sys.argv[2]
        merge_rosbag_topics(bag_file, output_file)
    else:
        print("Usage: python3 bag_2_csv.py [<bag_file.bag>] [<output_file.csv>]")

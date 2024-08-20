import os
import pandas as pd
import sys

def process_csv(input_file, output_file):
    # Construct the full path to the bag file
    csv_directory = os.path.join("..", "bagfiles", os.path.splitext(input_file)[0])
    csv_file_path = os.path.join(csv_directory, input_file)

    # Load the CSV file
    data = pd.read_csv(csv_file_path)

    # Identify rows where "goal_status" changes from 2 to 3
    start_index = data[data['goal_status_data'] == 2].index[0]
    print("Starting at index : ", start_index)
    end_data = data[data.index > start_index]  # Data after start_index

    # Check if there is any row where 'goal_status' is 3 after start_index
    if not end_data[end_data['goal_status_data'] == 3].empty:
        end_index = end_data[end_data['goal_status_data'] == 3].index[0]
    else:
        end_index = data.index[-1]  # Use the last index if no row meets the condition
    print("Ending at index : ", end_index)

    # Filter the DataFrame from start_index to end_index
    filtered_data = data.loc[start_index:end_index]

    # Offset the data by the first row's values
    reference_row = filtered_data.iloc[0]
    for column in filtered_data.columns:
        if column != 'goal_status_data':
            filtered_data[column] = filtered_data[column] - reference_row[column]

    # Drop the 'goal_status' column
    filtered_data = filtered_data.drop(columns=['goal_status_data'])

    # Save the processed data to a new CSV file
    filtered_data.to_csv(output_file, index=False)
    print(f"Processed data saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        default_csv_file = "report_sim7_obstacle0_rl0.csv"
        default_csv_file_without_extension = os.path.splitext(default_csv_file)[0]
        default_output_file = f"../bagfiles/{default_csv_file_without_extension}/{default_csv_file_without_extension}_filtered.csv"
        process_csv(default_csv_file, default_output_file)
    elif len(sys.argv) == 2:
        csv_file = sys.argv[1]
        default_csv_file_without_extension = os.path.splitext(csv_file)[0]
        default_output_file = f"../bagfiles/{default_csv_file_without_extension}/{default_csv_file_without_extension}_filtered.csv"
        process_csv(csv_file, default_output_file)
    elif len(sys.argv) == 3:
        csv_file = sys.argv[1]
        output_file = sys.argv[2]
        process_csv(csv_file, output_file)
    else:
        print("Usage: python filter_csv_data.py [<input_csv_file.csv>] [<output_csv_file.csv>]")

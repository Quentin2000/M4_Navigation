import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_data_from_csv(csv_file):
    # Extract the base name of the CSV file without the extension
    csv_file_without_extension = os.path.splitext(csv_file)[0]

    # Load the CSV file into a DataFrame
    base_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    csv_file_path = os.path.join(base_directory, "bagfiles", csv_file_without_extension, csv_file)

    data = pd.read_csv(csv_file_path)
    
    run_names = data['Run'].tolist()  # Convert run names to a list for easier plotting
    
    plot_directory = os.path.join(base_directory, "bagfiles", csv_file_without_extension)
    
    # Ensure the directory exists
    if not os.path.exists(plot_directory):
        os.makedirs(plot_directory)

    # Iterate over all columns except the first one which is 'Run'
    for column in data.columns[1:]:
        plt.figure()
        plt.plot(run_names, data[column].values, marker='o', linestyle='-')  # Ensure to pass numpy array
        plt.xlabel('Run Name')
        plt.ylabel(column)
        plt.title(f"{column} per Run")
        plt.xticks(rotation=80)  # Rotate labels to prevent overlap
        plt.tight_layout()  # Adjust layout to make room for label rotation
        
        # Define the plot filename
        plot_filename = os.path.join(plot_directory, f"{column}.png")
        
        # Save the plot as a PNG file
        plt.savefig(plot_filename)
        plt.close()  # Close the plot to free up memory

if __name__ == '__main__':
    if len(sys.argv) == 1:
        csv_file = 'report_sim6_obstacle0_rl.csv'
        plot_data_from_csv(csv_file)
    elif len(sys.argv) == 2:
        csv_file = sys.argv[1]  # The pattern provided from the command line
        plot_data_from_csv(csv_file)
    else:
        print("Usage: python plot_csv_data.py <name_of_the_csv> (Example: report_sim6_obstacle0_rl.csv)")

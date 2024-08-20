import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_data_from_csvs(csv_files):
    base_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']  # Define some colors for the plots
    
    # Iterate over all columns except the first one which is assumed to be 'Run' or similar non-numeric
    first_file = True
    for i, csv_file in enumerate(csv_files):
        csv_file_without_extension = os.path.splitext(csv_file)[0]
        csv_file_path = os.path.join(base_directory, "bagfiles", csv_file_without_extension, csv_file)
        data = pd.read_csv(csv_file_path)

        if first_file:
            # Create plot directory based on the first file
            plot_directory = os.path.join(base_directory, "bagfiles", "comparison_plots")
            if not os.path.exists(plot_directory):
                os.makedirs(plot_directory)
            columns = data.columns[1:]  # Assuming first column is 'Run' or similar
            first_file = False

        # Determine color for this file's plots
        color = colors[i % len(colors)]
        
        # Plot each column's data
        for column in columns:
            plt.figure(column, figsize=(10, 8))
            plt.plot(data[column].values, marker='o', linestyle='-', color=color, label=f"{csv_file_without_extension}")
            plt.xlabel('Row Index')
            plt.ylabel(column)
            plt.title(f"{column} Comparison Across Runs")
            plt.legend()

    # Save the plots for each column
    for column in columns:
        plt.figure(column)
        plt.xticks(rotation=80)  # Rotate labels to prevent overlap
        plt.tight_layout()  # Adjust layout to make room for label rotation
        plt.savefig(os.path.join(plot_directory, f"{column}.png"))
        plt.close()  # Close the plot to free up memory

if __name__ == '__main__':
    if len(sys.argv) > 1:
        csv_files = sys.argv[1:]  # All arguments are considered CSV files
        plot_data_from_csvs(csv_files)
    else:
        print("Usage: python plot_csv_data.py <csv_file1> <csv_file2> ... (Example: report_sim6_obstacle0_rl.csv report_sim7_obstacle0_rl.csv)")

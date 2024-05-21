import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def display_statistics(csv_file):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(csv_file)
    
    # Check if there are at least two columns in the CSV file
    if df.shape[1] < 2:
        print("Error: The CSV file must contain at least two columns.")
        return
    
    columns = df.columns[1:]
    means = []
    std_devs = []

    # Calculate Z-score for 98% confidence interval
    z_score_98 = 2.33
    
    # Calculate mean and standard deviation for each column starting from the second one
    for column in columns:
        mean_value = df[column].mean()
        std_deviation = df[column].std()
        means.append(mean_value)
        std_devs.append(std_deviation)
        print(f"Column: {column}")
        print(f"  Mean: {mean_value}")
        print(f"  Standard Deviation: {std_deviation}\n")
        print(f"  Upper bound 98% confidence interval: {mean_value + z_score_98 * std_deviation}\n")
        print(f"  Lower bound 98% confidence interval: {mean_value - z_score_98 * std_deviation}\n")

    # Calculate the lower and upper bounds for the 98% confidence interval
    lower_bound_98 = np.array(means) - z_score_98 * np.array(std_devs)
    upper_bound_98 = np.array(means) + z_score_98 * np.array(std_devs)
    
    # Plot the statistics
    x = range(len(columns))
    plt.errorbar(x, means, yerr=std_devs, fmt='o', capsize=5, capthick=2, ecolor='red', marker='o', color='black', linestyle='None', label='Mean ± 1 Std Dev')
    
    # Plot the 98% confidence interval
    plt.errorbar(x, means, (upper_bound_98+lower_bound_98)/2, fmt='o', capsize=5, capthick=2, ecolor='blue', marker='o', color='black', linestyle='None', label='98% Confidence Interval')
    
    # Add annotations for mean and standard deviation
    for i in range(len(columns)):
        plt.text(x[i]+0.05, means[i], f'Mean: {means[i]:.4f}\nSD: {std_devs[i]:.4f}\n98%: {z_score_98 * std_devs[i]:.4f}', 
                 ha='left', va='center', fontsize=8, color='black', 
                 bbox=dict(facecolor='white', alpha=0.6, edgecolor='none'))

    plt.xlabel('Columns')
    plt.ylabel('Values')
    plt.title('Mean, Standard Deviation, and 98% Confidence Interval of CSV Columns')
    plt.xticks(x, columns, rotation=45)
    plt.legend()
    plt.tight_layout()
    
    # Extract directory path from the CSV file path
    output_directory = os.path.dirname(csv_file)
    
    # Save the graph in the same folder as the CSV file
    graph_file = os.path.join(output_directory, 'statistics_plot.png')
    plt.savefig(graph_file)
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) == 1:
        csv_file = f"../bagfiles/speed_norm_idle/speed_norm_idle.csv"
        display_statistics(csv_file)
    elif len(sys.argv) == 2:
        bag_file = sys.argv[1]
        default_bag_file_without_extension = os.path.splitext(bag_file)[0]
        csv_file = f"../bagfiles/{default_bag_file_without_extension}/{default_bag_file_without_extension}.csv"
        display_statistics(csv_file)
    else:
        print("Usage: python display_statistics.py <csv_file>")

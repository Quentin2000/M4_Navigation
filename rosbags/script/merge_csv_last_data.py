import os
import pandas as pd
import sys

def compile_report_csv(pattern):
    # Base directory is one level above the script directory
    base_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bagfiles_path = os.path.join(base_directory, "bagfiles")
    
    if not os.path.exists(bagfiles_path):
        print(f"Directory not found: {bagfiles_path}")
        return
    
    # Prepare the output DataFrame to store the last rows of each CSV
    final_data = pd.DataFrame()

    # Iterate over subdirectories that match the pattern
    for i in range(10):
        folder_name = f"{pattern}{i}"
        folder_path = os.path.join(bagfiles_path, folder_name)
        
        if os.path.isdir(folder_path):
            csv_file_name = f"{folder_name}_filtered.csv"
            csv_file_path = os.path.join(folder_path, csv_file_name)
            
            # Check if the CSV file exists
            if os.path.isfile(csv_file_path):
                # Read the CSV file
                data = pd.read_csv(csv_file_path)
                
                # Extract the last row
                if not data.empty:
                    final_row = data.iloc[[-1]]
                    final_row['Run'] = folder_name  # Add folder info as a new column
                    # Reorder columns to put 'Folder' at the front
                    cols = final_row.columns.tolist()
                    cols = [cols[-1]] + cols[:-1]  # Move 'Folder' to the first position
                    final_row = final_row[cols]
                    final_data = pd.concat([final_data, final_row], ignore_index=True)
            else:
                print(f"CSV file not found: {csv_file_path}")

    # Write the compiled data to a new CSV file using the pattern as the filename
    output_folder_path = os.path.join(bagfiles_path, pattern)
    
    # Ensure the directory exists
    if not os.path.exists(output_folder_path):
        os.makedirs(output_folder_path)

    output_path = os.path.join(output_folder_path, f"{pattern}.csv")
    
    final_data.to_csv(output_path, index=False)
    print(f"Compiled data saved to {output_path}")


if __name__ == '__main__':
    if len(sys.argv) == 1:
        pattern = "report_sim6_obstacle0_rl"
        compile_report_csv(pattern)
    elif len(sys.argv) == 2:
        pattern = sys.argv[1]  # The pattern provided from the command line
        compile_report_csv(pattern)
    else:
        print("Usage: python merge_csv_last_data.py.py <name_of_the_report> (Example: report_sim6_obstacle0_rl)")

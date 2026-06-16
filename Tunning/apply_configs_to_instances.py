import random
from collections import defaultdict
import sys
import os

def apply_configs_to_instances(selected_configs_path, input_instance_dir, output_instance_dir):
    """
    Reads selected configurations from a file, modifies corresponding instance files,
    and saves the modified files to an output directory. If an instance has multiple
    configurations, multiple copies of the instance file are created.

    Args:
        selected_configs_path (str): Path to the file containing selected configurations (e.g., selected_configs.txt).
        input_instance_dir (str): Path to the directory containing original instance files.
        output_instance_dir (str): Path to the directory where modified instance files will be saved.
    """
    instance_configs_map = defaultdict(list)
    config_header = [] # To store the header from selected_configs.txt

    # 1. Read selected_configs.txt to get instance names and their configurations
    try:
        with open(selected_configs_path, 'r', encoding='utf-8') as f:
            header_line = f.readline().strip()
            config_header = header_line.split('\t')
            
            if not config_header or config_header[0] != 'name':
                print(f"Error: Invalid header in {selected_configs_path}. Expected 'name' as the first column.", file=sys.stderr)
                return

            # Parameters to be inserted into the instance file (excluding the 'name' column)
            param_names_to_insert = config_header[1:] 

            for line in f:
                parts = line.strip().split('\t')
                parts = [p for p in parts if p] # Filter out any empty strings from split
                if len(parts) == len(config_header):
                    instance_name = parts[0]
                    config_dict = dict(zip(config_header, parts))
                    instance_configs_map[instance_name].append(config_dict)
                elif len(parts) > 0:
                    print(f"Warning: Skipping malformed line in {selected_configs_path}: {line.strip()}", file=sys.stderr)
    except FileNotFoundError:
        print(f"Error: Selected configurations file not found at {selected_configs_path}", file=sys.stderr)
        return
    except Exception as e:
        print(f"An error occurred while reading {selected_configs_path}: {e}", file=sys.stderr)
        return

    if not instance_configs_map:
        print(f"Warning: No configurations loaded from {selected_configs_path}. No files will be processed.", file=sys.stderr)
        return

    # 2. Create the output directory if it doesn't exist
    os.makedirs(output_instance_dir, exist_ok=True)
    print(f"Output directory '{output_instance_dir}' ensured.")

    # 3. Process each instance and its configurations
    for instance_name, configs_list in instance_configs_map.items():
        original_instance_file_path = os.path.join(input_instance_dir, f"{instance_name}.inst")

        if not os.path.exists(original_instance_file_path):
            print(f"Warning: Original instance file not found for '{instance_name}' at {original_instance_file_path}. Skipping this instance.", file=sys.stderr)
            continue

        try:
            with open(original_instance_file_path, 'r', encoding='utf-8') as f_orig:
                original_lines = f_orig.readlines()
        except Exception as e:
            print(f"Error: Could not read original instance file {original_instance_file_path}: {e}. Skipping this instance.", file=sys.stderr)
            continue

        # Iterate through each configuration for the current instance
        for i, config_dict in enumerate(configs_list):
            # Construct the parameter string to insert (values only, tab-separated)
            # The order of parameters is crucial and should match the header from instance_parameters.txt
            param_values = [config_dict.get(param, '') for param in param_names_to_insert]
            parameter_string = '\t'.join(param_values)

            # Construct the new content: insert the parameter string at the second line
            if len(original_lines) >= 1:
                # Insert after the first line (at index 1)
                new_lines = original_lines[:1] + [parameter_string + '\n'] + original_lines[1:]
            else:
                # If the original file is empty, just add the parameter string
                new_lines = [parameter_string + '\n']

            # Determine the output filename. Append a counter if there are multiple configs for the same instance.
            if len(configs_list) == 1:
                output_filename = f"{instance_name}.inst"
            else:
                output_filename = f"{instance_name}_{i+1}.inst" # Use 1-based index for copies

            output_file_path = os.path.join(output_instance_dir, output_filename)

            try:
                with open(output_file_path, 'w', encoding='utf-8') as f_out:
                    f_out.writelines(new_lines)
                print(f"Generated modified instance file: {output_file_path}")
            except Exception as e:
                print(f"Error: Could not write to output file {output_file_path}: {e}", file=sys.stderr)

if __name__ == "__main__":
    # Define the paths
    selected_configs_file = "E:/Code/MyCode/OP_mD_BC/Tunning/selected_configs.txt"
    input_instances_folder = "E:/Code/MyCode/OP_mD_BC/Tunning/instance"
    output_instances_folder = "E:/Code/MyCode/OP_mD_BC/Tunning/output_instance"
    
    apply_configs_to_instances(selected_configs_file, input_instances_folder, output_instances_folder)

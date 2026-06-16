import random
from collections import defaultdict
import sys

def select_random_configurations(filepath, num_selections=4, output_filepath="selected_configs.txt"):
    """
    Reads instance parameters from a file, groups them by instance name,
    then randomly selects a specified number of configurations for each instance,
    and writes the results to an output file in a table format.

    Args:
        filepath (str): The path to the instance_parameters.txt file.
        num_selections (int): The number of random configurations to select per instance.
        output_filepath (str): The path to the output file where results will be written.
    """
    instance_configs = defaultdict(list)
    header = []

    try:
        with open(filepath, 'r', encoding='utf-16') as f:
            header_line = f.readline().strip()
            header = header_line.split('\t')
            
            for line in f:
                parts = line.strip().split('\t')
                parts = [p for p in parts if p] 
                if len(parts) == len(header):
                    instance_name = parts[0]
                    # Create a dictionary for the config, including the instance name
                    full_config = dict(zip(header, parts))
                    instance_configs[instance_name].append(full_config)
                elif len(parts) > 0:
                    print(f"Warning: Skipping malformed line: {line.strip()}", file=sys.stderr)
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}", file=sys.stderr)
        return
    except UnicodeDecodeError:
        print(f"Error: Could not decode file with utf-16 encoding. Please check the file encoding of {filepath}.", file=sys.stderr)
        return
    except Exception as e:
        print(f"An unexpected error occurred while reading the file: {e}", file=sys.stderr)
        print(f"Please ensure '{filepath}' is a plain text file and check its encoding.", file=sys.stderr)
        return

    if not instance_configs:
        print(f"Warning: No configurations were loaded from '{filepath}'. This might be due to an incorrect file format or encoding issues.", file=sys.stderr)
        print("Please verify the file content and its encoding.", file=sys.stderr)
        return

    with open(output_filepath, 'w', encoding='utf-8') as outfile:
        # Write header to the output file
        outfile.write('\t'.join(header) + '\n')

        for instance, configs in instance_configs.items():
            if len(configs) > num_selections:
                selected_configs = random.sample(configs, num_selections)
            else:
                selected_configs = configs

            for config in selected_configs:
                # Write each config as a tab-separated line
                row_values = [str(config.get(col, '')) for col in header]
                outfile.write('\t'.join(row_values) + '\n')
    
    print(f"Selected configurations have been written to {output_filepath}")


if __name__ == "__main__":
    # 对每个算例随机选择其中四个参数配置
    file_path = "E:/Code/MyCode/OP_mD_BC/Tunning/instance_parameters.txt"
    output_file = "E:/Code/MyCode/OP_mD_BC/Tunning/selected_configs.txt"
    select_random_configurations(file_path, output_filepath=output_file)

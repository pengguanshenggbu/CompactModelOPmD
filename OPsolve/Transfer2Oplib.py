import os
import sys

# Allow running this script from OPsolve/ directly.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Utils.Utils import parse_instance_file
from Utils.GetTmax import Get_Tmax

def get_cost_limit(param_file, instance_name):
    try:
        with open(param_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except UnicodeDecodeError:
         with open(param_file, 'r', encoding='utf-16') as f:
            lines = f.readlines()
            
    header = lines[0].strip().split('\t')
    
    try:
        name_col_index = header.index('name')
        t_col_index = header.index('T')
    except ValueError:
        return 0

    for line in lines[1:]:
        parts = line.strip().split('\t')
        if len(parts) > max(name_col_index, t_col_index):
            if parts[name_col_index] == instance_name:
                return float(parts[t_col_index])
    return 0

# def get_cost_limt_ratio(T_ratio):
#     name = f"c1-{n}-{i}"
#     coords, p = parse_instance_file(dir + name + '.inst')
#     N = list(coords.keys())  # 点索引集合
#     para = prepare_parameters(coords, 1, N, m, p, beta, L_ratio, T_ratio)

def convert_inst_to_oplib(inst_file, oplib_file, cost_limit):
    instance_name = os.path.splitext(os.path.basename(inst_file))[0]
    # cost_limit = get_cost_limit(param_file, instance_name)

    with open(inst_file, 'r') as f:
        lines = f.readlines()

    if not lines:
        print("Empty .inst file.")
        return

    num_nodes = int(lines[0].strip())
    
    with open(oplib_file, 'w') as out_f:
        out_f.write(f"NAME : {instance_name}\n")
        out_f.write(f"COMMENT : Converted from {instance_name}.inst\n")
        out_f.write(f"TYPE : OP\n")
        out_f.write(f"DIMENSION: {num_nodes}\n")
        out_f.write(f"COST_LIMIT : {cost_limit}\n")
        out_f.write(f"EDGE_WEIGHT_TYPE : EUC_2D\n")
        
        out_f.write("NODE_COORD_SECTION\n")
        nodes_data = []
        for i, line in enumerate(lines[1:num_nodes+1]):
            parts = line.strip().split()
            if len(parts) >= 3:
                x, y, score = parts[0], parts[1], parts[2]
                node_id = i + 1
                out_f.write(f"{node_id} {x} {y}\n")
                nodes_data.append((node_id, score))
                
        out_f.write("NODE_SCORE_SECTION\n")
        for node_id, score in nodes_data:
            out_f.write(f"{node_id} {score}\n")
            
        out_f.write("DEPOT_SECTION\n")
        out_f.write("1\n")
        out_f.write("-1\n")
        out_f.write("EOF\n")

    # print(f"Successfully converted {inst_file} to {oplib_file}")



if __name__ == "__main__":

    Ns = [50]
    T_ratios = [0.6]
    IndexRange = range(1, 6) # 同一规模下算例的标识

    for Num in Ns:
        for i in IndexRange:
            for T_ratio in T_ratios:
                file_name = f'poi-{Num}-{i}'
                inst_file_path = os.path.join(PROJECT_ROOT, "Data", "random", f"{file_name}.inst")

                output_dir = os.path.join(PROJECT_ROOT, "OPsolve", f"{T_ratio}T")
                if not os.path.exists(output_dir):
                    os.makedirs(output_dir)
                oplib_file_path = os.path.join(output_dir, f"{file_name}.oplib")

                # 计算TSP
                coords, p = parse_instance_file(inst_file_path)
                para = Get_Tmax(coords, T_ratio)

                Tmax = para.Tmax
                output_str = f"{file_name}\t{T_ratio}\t{Tmax}"
                print(output_str)
                with open(os.path.join(PROJECT_ROOT, "OPsolve", "Tmax_results.txt"), 'a') as f:
                    f.write(output_str + "\n")
                convert_inst_to_oplib(inst_file_path, oplib_file_path, Tmax)



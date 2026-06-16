import os
import csv
import pandas as pd
from numpy.ma.extras import average

input_dir = os.path.dirname(__file__)
# output_file = os.path.join(input_dir, 'max.txt')

file_num = 1
# 读取所有文件内容
data_files = [os.path.join(input_dir, f'{i}.txt') for i in range(0, file_num)]
all_lines = []
headers = None
for file in data_files:
    with open(file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        if headers is None:
            headers = lines[0].strip()  # 取表头
        all_lines.append([line.strip().split() for line in lines[1:]])  # 跳过表头

# 假设每个文件行数一致
num_rows = len(all_lines[0])

# 输出value 最大的行到新文件
# with open(output_file, 'w', encoding='utf-8') as out:
#     out.write(headers + '\n')
#     for row_idx in range(num_rows):
#         # 取5个文件该行的第一个元素及其后三个元素
#         candidates = [(float(all_lines[file_idx][row_idx][0]), all_lines[file_idx][row_idx][1:]) for file_idx in range(5)]
#         max_val, rest = max(candidates, key=lambda x: x[0])
#         out.write(f"{max_val} {' '.join(rest)}\n")

# 获得多次重复试验结果的最优值
max_result = []
min_result = []
average_result = []
for row_idx in range(num_rows-1):
    # 取5个文件该行的第一个元素及其后三个元素
    candidates = [all_lines[file_idx][row_idx] for file_idx in range(file_num)]
    max_row = max(candidates, key=lambda x: float(x[6]))
    max_result.append(max_row)

    min_row = min(candidates, key= lambda x:float(x[6]))
    min_result.append(min_row)

    # 前0-5列直接取第一个
    average_row = candidates[0][0:6]
    # 6-9列取平均
    avg_vals = []
    for col in range(6, 10):
        vals = [float(row[col]) for row in candidates]
        avg = sum(vals) / len(vals)
        avg_vals.append(f"{avg:.6f}")
    average_row += avg_vals
    average_result.append(average_row)
    # print(max_result[-1])

df_max = pd.DataFrame(max_result, columns=headers.split())
df_max.columns = ['InstanceName'] + df_max.columns[1:].tolist()
print("num_nodes\tnum_drones\tnum_solved\tnum_optimal\tGap_max\tCpu\tnum_unsolved\tnum_best\tGap_BC\tGap_unsolved\tCpu_unsolved\tcpu_bc")


optimal_file = os.path.join(input_dir, 'MainComputationsReport.csv')

Gap_max = []
# Gap_min = []
# Gap_average = []
# time_max = []
time2best_max = []

Gap_BC_unsolved = []
Gap_max_unsolved = []
time2best_max_unsolved = []


with open(optimal_file, 'r', encoding='utf-8') as f:
    reader = csv.reader(f)
    lines = list(reader)[1:]
    num_node = 10
    num_drone = 1


    for num_node in range(10, 60, 10):
        for num_drone in range(1,4):
            Gap_max = []
            time2best_max = []

            Gap_BC_unsolved = []
            Gap_max_unsolved = []
            time2best_max_unsolved = []
            time_BC = []

            num_solved = 0
            num_unsolved = 0
            num_optimal = 0
            num_best = 0

            result_node_drone = []
            for idx in range(1,6):
                name = 'poi-' + str(num_node) + '-' + str(idx)
                filtered_result = [line for line in max_result if line[0] == name and int(line[4]) == num_drone]
                result_node_drone += filtered_result
            for result_line in result_node_drone:
                # 匹配lines中前六项相同的行
                matched_lines = [l for l in lines if l[:6] == result_line[:6]]
                if len(matched_lines)>1 or len(matched_lines) == 0:
                    raise ValueError(f"数据不一致: 找到BC结果 {len(matched_lines)} 行匹配 {line[:6]}，期望1行")
                line = matched_lines[0]

                if line[7] == '-':
                    num_solved += 1
                    num_optimal += 1
                    continue  # 当前算例无可行解;仍计入已解决的数量，但不计算Gap

                if float(line[7]) == 0.0 :
                    num_solved += 1
                    optimal_value = float(line[6])
                    max_value = float(result_line[6])
                    if optimal_value == max_value:
                        num_optimal += 1

                    gap_max = (optimal_value - max_value) / optimal_value
                    Gap_max.append(gap_max)
                    time2best_max.append(float(result_line[9]))
                    time_BC.append(float(line[8]))
                else:
                    num_unsolved += 1
                    best_value = float(line[6])
                    upBound_value = float(line[10])
                    max_value = float(result_line[6])
                    gap_bc = (upBound_value - best_value) / best_value
                    gap_max = (upBound_value - max_value) / max_value
                    if max_value >= best_value:
                        num_best += 1

                    Gap_max_unsolved.append(gap_max)
                    time2best_max_unsolved.append(float(result_line[9]))
                    if best_value <= 0:
                        continue # BC算法在规定时间内找不到可行解；忽略计算其Gap
                    Gap_BC_unsolved.append(gap_bc)


            gap_solved = round(sum(Gap_max)/len(Gap_max) if len(Gap_max) > 0 else 0, 4)
            gap_bc_unsolved = round((sum(Gap_BC_unsolved) / len(Gap_BC_unsolved) if len(Gap_BC_unsolved) > 0 else 0), 4)

            gap_max_unsolved = round((sum(Gap_max_unsolved) / len(Gap_max_unsolved) if len(Gap_max_unsolved) > 0 else 0), 4)
            time2best_solved = round((sum(time2best_max) / len(time2best_max) if len(time2best_max) > 0 else 0), 4)
            time2best_unsolved = sum(time2best_max_unsolved) / len(time2best_max_unsolved) if len(time2best_max_unsolved) > 0 else 0
            time_bc_solved = sum(time_BC) / len(time_BC) if len(time_BC) > 0 else 0
            print(f"{num_node}\t{num_drone}\t{num_solved}\t{num_optimal}\t{gap_solved * 100:.2f}\t{time2best_solved:.6f}"
                  f"\t{num_unsolved}\t{num_best}\t{gap_bc_unsolved*100:.2f}\t{gap_max_unsolved*100:.2f}\t{time2best_unsolved:.6f}\t{time_bc_solved:.6f}")

    # for line_idx, line in enumerate(lines):
    #     if line_idx == 0:
    #         continue
    #     max_line = max_result[line_idx]
    #     # min_line = min_result[line_idx]
    #     # average_line = average_result[line_idx]
    #     if line[0:6] != max_line[0:6]:
    #         raise ValueError(f"数据不一致: {line[0:6]} != {max_line[0:6]}")
    #     if line[6] == '-':
    #         continue # 无可行解
    #     temp = int(line[0].split('-')[1])
    #     # temp_m = line[4]
    #     # 按节点数和无人机数量分组统计
    #     if temp != num_node or num_drone != int(line[4]):
    #
    #         num_solved = len(Gap_max)
    #         num_unsolved =len(Gap_max_unsolved)
    #
    #         gap_bc_unsolved = sum(Gap_BC_unsolved) / len(Gap_BC_unsolved) if len(Gap_BC_unsolved) > 0 else 0
    #         gap_max_unsolved = sum(Gap_max_unsolved) / num_unsolved if num_unsolved > 0 else 0
    #         time2best_unsolved = sum(time2best_max_unsolved) / num_unsolved if num_unsolved > 0 else 0
    #         print(f"{num_node}\t{num_drone}\t{num_solved}\t{num_optimal}\t{sum(Gap_max)/num_solved:.4f}\t{sum(time2best_max) / num_solved:.4f}"
    #               f"\t{num_best}\t{gap_bc_unsolved:.4f}\t{gap_max_unsolved:.4f}\t{time2best_unsolved:.4f}")
    #
    #         num_node = temp
    #         num_drone = int(line[4])
    #         Gap_max.clear()
    #         # Gap_min.clear()
    #         # Gap_average.clear()
    #         # time_max.clear()
    #         time2best_max.clear()
    #         Gap_BC_unsolved.clear()
    #         Gap_max_unsolved.clear()
    #         time2best_max_unsolved.clear()
    #         num_solved = 0
    #         num_unsolved = 0
    #         num_optimal = 0
    #         num_best = 0
    #
    #     if float(line[7]) == 0.0:
    #         optimal_value = float(line[6])
    #         max_value = float(max_line[6])
    #         if optimal_value == max_value:
    #             num_optimal += 1
    #
    #         # min_value = float(min_line[6])
    #         # average_value = float(average_line[6])
    #         gap_max = (optimal_value - max_value) / optimal_value
    #         # gap_min = (optimal_value - min_value) / optimal_value
    #         # gap_average = (optimal_value - average_value) / optimal_value
    #         Gap_max.append(gap_max)
    #         # Gap_min.append(gap_min)
    #         # Gap_average.append(gap_average)
    #         # time_max.append(float(max_line[7]))
    #         time2best_max.append(float(max_line[9]))
    #
    #     else:
    #         best_value = float(line[6])
    #         upBound_value = float(line[10])
    #         max_value = float(max_line[6])
    #         gap_bc = (upBound_value - best_value) / best_value
    #         gap_max = (upBound_value - max_value) / max_value
    #         if max_value >= best_value:
    #             num_best += 1
    #
    #         Gap_max_unsolved.append(gap_max)
    #         time2best_max_unsolved.append(float(max_line[9]))
    #         if best_value <= 0:
    #             continue # BC算法找不到可行解；忽略计算其Gap
    #         Gap_BC_unsolved.append(gap_bc)







import os
import csv
import pandas as pd
from numpy.ma.extras import average

# input_dir = os.path.dirname(__file__)
# optimal_file = os.path.join(input_dir, 'MainComputationsReport.csv')
# max_file = os.path.join(input_dir, 'best.txt')
# average_file = os.path.join(input_dir, 'average.txt')
# 读取max_file
# with open(max_file, 'r', encoding='utf-8') as f:
#     reader = csv.reader(f, delimiter='\t')
#     max_result = list(reader)
#
# with open(average_file, 'r', encoding='utf-8') as f:
#     reader = csv.reader(f, delimiter='\t')
#     average_result = list(reader)
# # 删除第一行
# max_result = max_result[1:]
# average_result = average_result[1:]
# 删除max_result 最后一行
# max_result = max_result[:-1]

def GetAllResultForEachSizeDroneNum(result, num_node, num_drone):
    # result: 求解结果：(name, alpha, beta, L_ratio, num_drone, Tmax, obj_value, CPU, IterToBest, TimeToBest)
    result_node_drone = []
    for idx in range(1, 6):
        name = 'poi-' + str(num_node) + '-' + str(idx)
        filtered_result_max = [line for line in result if
                               line[0] == name and int(line[4]) == num_drone]  # 按算例名称和无人机数量进行分组统计
        result_node_drone += filtered_result_max # [[poi-10-1,...,1,...],[poi-10-2,...,1,...]]]
    return result_node_drone

def GetSummerizeResult(max_result, average_result):
    with open(optimal_file, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        lines_optimal = list(reader)[1:] # 删除第一行

        CPU_bc_solved_total = 0
        CPU_vns_solved_total = 0
        num_solved_total = 0
        gap_solved_best_total = 0
        gap_solved_average_total = 0
        CPU_vns_unsolved_total = 0
        gap_unsolved_best_total = 0
        gap_unsolved_average_total = 0
        gap_unsolved_average_bc_total = 0
        gap_bc_unsolved_total = 0
        num_bc_unsolved_total = 0
        num_vns_unsolved_total = 0

        for num_node in range(10, 60, 10):
            for num_drone in range(1,4):
                Gap_max = []
                Gap_average = []
                time2best_average = []

                Gap_BC_unsolved = []
                Gap_max_unsolved = []
                Gap_average_unsolved = []
                Gap_average_bc_unsolved = []
                time2best_max_unsolved = []
                time_BC = []

                num_solved = 0
                num_unsolved = 0
                num_optimal = 0
                num_best = 0

                result_node_drone_max = GetAllResultForEachSizeDroneNum(max_result, num_node, num_drone)
                result_node_drone_average = GetAllResultForEachSizeDroneNum(average_result, num_node, num_drone)
                # line_num = 0
                for result_line_max in result_node_drone_max:
                    # 匹配lines中前六项相同的行
                    matched_lines_optimal = [l for l in lines_optimal if l[:6] == result_line_max[:6]]
                    if len(matched_lines_optimal)>1 or len(matched_lines_optimal) == 0:
                        raise ValueError(f"数据不一致: 找到BC结果 {len(matched_lines_optimal)} 行匹配 {line_optimal[:6]}，期望1行")
                    line_optimal = matched_lines_optimal[0]

                    result_line_average =  [l for l in result_node_drone_average if l[:6] == result_line_max[:6]]
                    if len(result_line_average)>1 or len(result_line_average) == 0:
                        raise ValueError(f"数据不一致: 找到BC结果 {len(result_line_average)} 行匹配 {result_line_average[:6]}，期望1行")
                    result_line_average = result_line_average[0]

                    if line_optimal[7] == '-': # 第7列为optimality gap;如果为'-'，表示当前算例无可行解；（模型禁止一个以下的非depot节点的解）
                        num_solved += 1
                        num_optimal += 1
                        continue  # 当前算例无可行解;仍计入已解决的数量，但不计算Gap

                    if float(line_optimal[7]) == 0.0 : # optimality gap = 0.0 该算例已求解
                        num_solved += 1
                        optimal_value = float(line_optimal[6])
                        max_value = float(result_line_max[6])
                        average_value = float(result_line_average[6])
                        if optimal_value == max_value:
                            num_optimal += 1 # 找到最优解的数量


                        gap_max = (optimal_value - max_value) / optimal_value
                        Gap_max.append(gap_max)
                        gap_average = (optimal_value - average_value) / optimal_value
                        Gap_average.append(gap_average)

                        time2best_average.append(float(result_line_average[9]))
                        time_BC.append(float(line_optimal[8]))
                        CPU_bc_solved_total += float(line_optimal[8])
                        num_solved_total += 1
                        gap_solved_best_total += gap_max
                        gap_solved_average_total += gap_average
                        CPU_vns_solved_total += float(result_line_average[9])
                    else:
                        num_unsolved += 1
                        best_value = float(line_optimal[6]) # solver best found value
                        upBound_value = float(line_optimal[10]) # solver upper bound
                        max_value = float(result_line_max[6])
                        average_value = float(result_line_average[6])
                        gap_bc = (upBound_value - best_value) / best_value
                        gap_max = (upBound_value - max_value) / max_value

                        gap_average = (upBound_value - average_value) / average_value
                        gap_average_bc = (average_value - best_value) / best_value

                        if max_value >= best_value:
                            num_best += 1

                        Gap_max_unsolved.append(gap_max)
                        Gap_average_unsolved.append(gap_average)
                        Gap_average_bc_unsolved.append(gap_average_bc)
                        time2best_max_unsolved.append(float(result_line_average[9]))

                        CPU_vns_unsolved_total += float(result_line_average[9])
                        gap_unsolved_best_total += gap_max
                        gap_unsolved_average_total += gap_average
                        gap_unsolved_average_bc_total += gap_average_bc
                        num_vns_unsolved_total +=1

                        if best_value <= 0:
                            continue # BC算法在规定时间内找不到可行解；忽略计算其Gap
                        Gap_BC_unsolved.append(gap_bc)
                        gap_bc_unsolved_total += gap_bc
                        num_bc_unsolved_total += 1

                    # line_num += 1

                gap_solved = round(sum(Gap_max)/len(Gap_max) if len(Gap_max) > 0 else 0, 4)
                gap_solved_average = round(sum(Gap_average) / len(Gap_average) if len(Gap_average) > 0 else 0, 4)
                gap_bc_unsolved = round((sum(Gap_BC_unsolved) / len(Gap_BC_unsolved) if len(Gap_BC_unsolved) > 0 else 0), 4)

                gap_max_unsolved = round((sum(Gap_max_unsolved) / len(Gap_max_unsolved) if len(Gap_max_unsolved) > 0 else 0), 4)
                gap_average_unsolved = round((sum(Gap_average_unsolved) / len(Gap_average_unsolved) if len(Gap_average_unsolved) > 0 else 0), 4)
                gap_average_bc_unsolved = round((sum(Gap_average_bc_unsolved) / len(Gap_average_bc_unsolved) if len(Gap_average_bc_unsolved) > 0 else 0), 4)
                time2best_solved = round((sum(time2best_average) / len(time2best_average) if len(time2best_average) > 0 else 0), 4)
                time2best_unsolved = sum(time2best_max_unsolved) / len(time2best_max_unsolved) if len(time2best_max_unsolved) > 0 else 0
                time_bc_solved = sum(time_BC) / len(time_BC) if len(time_BC) > 0 else 0
                print(f"{num_node}\t{num_drone}\t{num_solved}\t{time_bc_solved:.6f}\t{num_optimal}\t{gap_solved * 100:.2f}\t{gap_solved_average * 100:.2f}\t{time2best_solved:.6f}"
                      f"\t{num_unsolved}\t{gap_bc_unsolved*100:.2f}\t{num_best}\t{gap_max_unsolved*100:.2f}\t{gap_average_unsolved*100:.2f}\t{gap_average_bc_unsolved*100:.2f}\t{time2best_unsolved:.6f}")
        print(f"Average\t{CPU_bc_solved_total/num_solved_total:.6f}\t{gap_solved_best_total/num_solved_total*100:.2f}\t{gap_solved_average_total/num_solved_total*100:.2f}"
              f"\t{CPU_vns_solved_total/num_solved_total:.6f}\t{gap_bc_unsolved_total/num_bc_unsolved_total*100:.2f}\t{gap_unsolved_best_total/num_vns_unsolved_total*100:.2f}"
              f"\t{gap_unsolved_average_total/num_vns_unsolved_total*100:.2f}\t{gap_unsolved_average_bc_total/num_vns_unsolved_total*100:.2f}\t{CPU_vns_unsolved_total/num_vns_unsolved_total:.6f}")

if __name__ == "__main__":
    file_max = "max.txt"
    file_average = "average.txt"
    with open(file_max, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        max_result = [line.strip().split() for line in lines[1:]] # 删除第一行
    with open(file_average, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        average_result = [line.strip().split() for line in lines[1:]] # 删除第一行

    input_dir = os.path.dirname(__file__)
    optimal_file = os.path.join(input_dir, 'MainComputationsReport.csv')
    GetSummerizeResult(max_result, average_result)
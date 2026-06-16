from VNS_result.computeGapforBestResult import GetAllResultForEachSizeDroneNum
import os
import csv


def GetAllResultForEachSize(result, num_node):
    # result: 求解结果：(name, alpha, beta, L_ratio, num_drone, Tmax, obj_value, CPU, IterToBest, TimeToBest)
    result_node = []
    for idx in range(1, 6):
        name = ''
        if num_node == 50:
            name = 'poi-' + str(num_node) + '-' + str(idx)
        if num_node > 50 :
            name = 'r-' + str(num_node) + '-' + str(idx)
        filtered_result = [line for line in result if
                               line[0] == name]  # 按算例名称
        result_node += filtered_result  # [[poi-10-1,...,1,...],[poi-10-2,...,1,...]]]
    return result_node

def GetAllFileNames(num_node):
    names = []
    for idx in range(1, 6):
        name = ''
        if num_node == 50:
            name = 'poi-' + str(num_node) + '-' + str(idx)
        if num_node > 50 :
            name = 'r-' + str(num_node) + '-' + str(idx)
        names.append(name)

    return names

def GetAllResultForEachSizePara(result, num_node, para, ind):
    # result: 求解结果：(name, alpha, beta, L_ratio, num_drone, Tmax, obj_value, CPU, IterToBest, TimeToBest)
    result_node = []
    for idx in range(1, 6):
        name = ''
        if num_node == 50:
            name = 'poi-' + str(num_node) + '-' + str(idx)
        if num_node > 50 :
            name = 'r-' + str(num_node) + '-' + str(idx)
        filtered_result = [line for line in result if
                               line[0] == name and line[ind] == para]  # 按算例名称
        result_node += filtered_result  # [[poi-10-1,...,1,...],[poi-10-2,...,1,...]]]
    return result_node


def GetNodeCompare(result_fast, result_full):
    print('NodeNum\tFull\tFast\tSpeedup')
    for num_node in range(50, 175, 25):
        # for num_drone in range(1, 4):

        result_node_drone_fast = GetAllResultForEachSize(result_fast, num_node)
        result_node_drone_full = GetAllResultForEachSize(result_full, num_node)

        times_fast = []
        times_full = []
        for  line_fast in result_node_drone_fast:
            matched_lines_full = [l for l in result_node_drone_full if l[:6] == line_fast[:6]]
            if len(matched_lines_full) > 1 or len(matched_lines_full) == 0:
                raise ValueError(
                    f"数据不一致: 找到BC结果 {len(matched_lines_full)} 行匹配 {line_fast[:6]}，期望1行")
            line_full = matched_lines_full[0]

            times_fast.append(float(line_fast[7]))
            times_full.append(float(line_full[7]))

        time_fast_average = round((sum(times_fast) / len(times_fast) if len(times_fast) > 0 else 0), 2)
        time_full_average = round((sum(times_full) / len(times_full) if len(times_full) > 0 else 0), 2)

        print(f"{num_node}\t{time_full_average}\t{time_fast_average}\t{time_full_average/time_fast_average:.2f}")

def GetCompare(result_fast, result_full, ParaName, rangaPara, index):

    print(f'{ParaName}\tFull\tFast\tSpeedup')
    for num in rangaPara:
        # for num_drone in range(1, 4):
        result_fast_filter = [line for line in result_fast if
                               line[index] == str(num)]
        result_full_filter = [line for line in result_full if
                               line[index] == str(num)]

        times_fast = []
        times_full = []
        for  line_fast in result_fast_filter:
            matched_lines_full = [l for l in result_full_filter if l[:6] == line_fast[:6]]
            if len(matched_lines_full) > 1 or len(matched_lines_full) == 0:
                raise ValueError(
                    f"数据不一致: 找到BC结果 {len(matched_lines_full)} 行匹配 {line_fast[:6]}，期望1行")
            line_full = matched_lines_full[0]

            times_fast.append(float(line_fast[7]))
            times_full.append(float(line_full[7]))

        time_fast_average = (sum(times_fast) / len(times_fast) if len(times_fast) > 0 else 0)
        time_full_average = (sum(times_full) / len(times_full) if len(times_full) > 0 else 0)

        print(f"{num}\t{time_full_average}\t{time_fast_average}\t{time_full_average/time_fast_average:.2f}")



if __name__ == "__main__":
    file_fast = "fastEvaluation.txt"
    file_full = "fullEvaluation.txt"

    with open(file_fast, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        result_fast = [line.strip().split() for line in lines[1:-1]] # 删除第一行和最后一行
    with open(file_full, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        result_full = [line.strip().split() for line in lines[1:-1]] # 删除第一行和最后一行

    GetNodeCompare(result_fast, result_full)
    GetCompare(result_fast, result_full, 'DroneNum', range(1,4),  4)



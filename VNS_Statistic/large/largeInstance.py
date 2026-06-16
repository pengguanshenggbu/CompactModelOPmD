file_insertion = "insertion.txt"
file_fast = "fast.txt"
file_full = "full.txt"

with open(file_insertion, 'r', encoding='utf-8') as f:
    lines = f.readlines()
    insertion_result = [line.strip().split() for line in lines[1:-1]]  # 删除第一行
with open(file_fast, 'r', encoding='utf-8') as f:
    lines = f.readlines()
    file_fast = [line.strip().split() for line in lines[1:-1]]
with open(file_full, 'r', encoding='utf-8') as f:
    lines = f.readlines()
    file_full = [line.strip().split() for line in lines[1:-1]]

def GetAllResultForEachSizeDroneNum(result, num_node, num_drone):
    # result: 求解结果：(name, alpha, beta, L_ratio, num_drone, Tmax, obj_value, CPU, IterToBest, TimeToBest)
    result_node_drone = []
    for idx in range(1, 6):
        name = 'r-' + str(num_node) + '-' + str(idx)
        filtered_result_max = [line for line in result if
                               line[0] == name and int(line[4]) == num_drone]  # 按算例名称和无人机数量进行分组统计
        result_node_drone += filtered_result_max # [[poi-10-1,...,1,...],[poi-10-2,...,1,...]]]
    return result_node_drone

for num_node in range(75, 175, 25):
    for num_drone in range(1, 4):
        Gap_insert = []
        Gap_fast = []
        Gap_full = []

        num_insert = 0 #该方法获得最好解的个数
        num_fast = 0
        num_full = 0

        time_insert = []
        time_fast = []
        time_full = []

        result_node_drone_insertion = GetAllResultForEachSizeDroneNum(insertion_result, num_node, num_drone)
        result_node_drone_fast = GetAllResultForEachSizeDroneNum(file_fast, num_node, num_drone)
        result_node_drone_full = GetAllResultForEachSizeDroneNum(file_full, num_node, num_drone)
        # line_num = 0
        for result_line_full in result_node_drone_full:
            # 匹配lines中前六项相同的行
            result_line_insert = [l for l in result_node_drone_insertion if l[:6] == result_line_full[:6]]
            if len(result_line_insert) > 1 or len(result_line_insert) == 0:
                raise ValueError(
                    f"数据不一致: 找到结果 {len(result_line_insert)} 行匹配 {result_line_insert[:6]}，期望1行")
            result_line_insert = result_line_insert[0]

            result_line_fast = [l for l in result_node_drone_fast if l[:6] == result_line_full[:6]]
            if len(result_line_fast) > 1 or len(result_line_fast) == 0:
                raise ValueError(
                    f"数据不一致: 找到结果 {len(result_line_fast)} 行匹配 {result_line_fast[:6]}，期望1行")
            result_line_fast = result_line_fast[0]

            # 找到解值最大的那个
            value_insert = float(result_line_insert[6])
            value_fast = float(result_line_fast[6])
            value_full = float(result_line_full[6])
            max_value = max(value_insert, value_fast, value_full)
            if(value_insert == max_value):
                num_insert += 1
            if(value_fast == max_value):
                num_fast += 1
            if(value_full == max_value):
                num_full += 1

            gap_insert = (max_value - value_insert) / max_value
            Gap_insert.append(gap_insert)

            gap_fast = (max_value - value_fast ) / max_value
            Gap_fast.append(gap_fast)

            gap_full = (max_value - value_full) / max_value
            Gap_full.append(gap_full)

            time_insert.append(float(result_line_insert[9]))
            time_fast.append(float(result_line_fast[9]))
            time_full.append(float(result_line_full[9]))


        gap_insert_average = round(sum(Gap_insert) / len(Gap_insert) if len(Gap_insert) > 0 else 0, 4)
        gap_fast_average = round(sum(Gap_fast) / len(Gap_fast) if len(Gap_fast) > 0 else 0, 4)
        gap_full_average = round((sum(Gap_full) / len(Gap_full) if len(Gap_full) > 0 else 0), 4)

        time_insert_average = round(sum(time_insert) / len(time_insert) if len(time_insert) > 0 else 0, 4)
        time_fast_average = round(sum(time_fast) / len(time_fast) if len(time_fast) > 0 else 0, 4)
        time_full_average = round(sum(time_full) / len(time_full) if len(time_full) > 0 else 0, 4)
        print(f"{num_node}\t{num_drone}\t{num_insert}\t{gap_insert_average * 100:.2f}\t{time_insert_average:.4f}\t{num_fast}\t{gap_fast_average * 100:.2f}\t"
              f"{time_fast_average:.4f}\t{num_full}\t{gap_full_average * 100:.2f}\t{time_full_average:.4f}")
import os
import csv
import pandas as pd
from numpy.ma.extras import average

input_dir = os.path.dirname(__file__)
# output_file = os.path.join(input_dir, 'max.txt')

file_num = 10
# 读取所有文件内容
data_files = [os.path.join(input_dir, f'{i}.txt') for i in range(0, file_num)]
all_lines = []
headers = None
for file in data_files:
    with open(file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        if headers is None:
            headers = lines[0].strip()  # 取表头
        all_lines.append([line.strip().split() for line in lines[1:-1]])  # 跳过表头

# 假设每个文件行数一致
num_rows = len(all_lines[0])

# 获得多次重复试验结果的最优值
max_result = []
# min_result = []
average_result = []
for row_idx in range(num_rows):
    # 取5个文件该行的第一个元素及其后三个元素
    candidates = [all_lines[file_idx][row_idx] for file_idx in range(file_num)]
    max_row = max(candidates, key=lambda x: float(x[6])) # 第6列为solution value
    max_result.append(max_row)

    # min_row = min(candidates, key= lambda x:float(x[6]))
    # min_result.append(min_row)

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

df_max = pd.DataFrame(max_result, columns=headers.split())
df_max.columns = ['InstanceName'] + df_max.columns[1:].tolist()
df_max.to_csv("max.txt", sep="\t", index=False)

df_average = pd.DataFrame(average_result, columns=headers.split())
df_average.columns = ['InstanceName'] + df_max.columns[1:].tolist()
df_average.to_csv("average.txt", sep="\t", index=False)
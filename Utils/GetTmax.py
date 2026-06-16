import math

from Utils.TSPmodel import TSP
from Utils.Utils import parse_instance_file, calculate_travel_times

import pandas as pd

class Get_Tmax:
    # 给定坐标点数据，通过计算TSP解获得TSP最优解，再乘以T_ratio得到Tmax
    def __init__(self, coords, T_ratio):
        # 输入参数
        self.coords = coords

        self.N = list(coords.keys())

        self.T_ratio = T_ratio

        # 输出参数
        self.truck_times, _, _ = calculate_travel_times(self.coords, alpha = 1.0) # 距离矩阵字典{(i,j)=t_{ij}}

        self.Tmax = self.build_T()


    def build_T(self):

        tsp=TSP(self.N,self.truck_times)

        tsp.solve()

        return math.floor(tsp.optimal_cost * self.T_ratio)


if __name__ == "__main__":
    dir = "./Data/random/" # "./baseline/instances/OP-mD/"
    type = 'r' # poi, r, c, rc

    # 根据初始算例数据和参数设定，通过计算TSP模型得到T， 输出最终的算例参数

    T_ratios = [0.5] # 最大完工时间按TSP最优解的倍数

    ns = [50, 100, 150]
    instance_ids = range(1, 6) # 同一规模下算例的标识

    params = {
        "name": [],
        "T_ratio": [],
        "T": []
    }

    for n in ns:
        for i in instance_ids:
            for T_ratio in T_ratios:
                name = f"{type}-{n}-{i}"
                coords, p = parse_instance_file(dir + name + '.inst')
                para = Get_Tmax(coords, T_ratio)

                # 记录所有参数和结果
                params["name"].append(name)
                params["T_ratio"].append(T_ratio)
                params["T"].append(para.Tmax)

                print(f"{name}\t{T_ratio}\t{para.Tmax}")


# df = pd.DataFrame(params)
# print(df.to_string(index=False))

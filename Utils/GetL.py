import math
from pathlib import Path

from TSPmodel import TSP
from Utils import parse_instance_file, calculate_travel_times

import pandas as pd

class get_L:

    def __init__(self, coords, N ,L_ratio ):
        # 输入参数
        self.coords = coords

        self.N = N

        self.L_ratio = L_ratio

        # 输出参数
        self.truck_times, _,  self.euclidean_times = calculate_travel_times(coords, alpha =1) # 距离矩阵字典{(i,j)=t_{ij}}

        self.A=self.truck_times.keys()

        self.AVG=self.calculate_AVG()

        self.L = self.calculate_endurance()


    def build_drone_times(self):

        return {h: self.drone_times_two[h[1],h[2]]+self.drone_times_two[h[0],h[1]] for h in self.H}



    def calculate_AVG(self):

        return sum([math.floor(x) for x in list(self.euclidean_times.values())]) / len(self.euclidean_times)


    def calculate_endurance(self):

        endurance = math.floor(self.AVG*self.L_ratio)

        return endurance


if __name__ == "__main__":
    data_root = Path(__file__).resolve().parent.parent / "Data"
    type = 'r' # poi, r, c, rc
    types = ['r', 'c', 'rc']

    def resolve_instance_path(instance_dir, base_type, n, i):
        if base_type == 'r' and n <= 50:
            candidates = [f"poi-{n}-{i}", f"r-{n}-{i}"]
        elif base_type == 'c':
            candidates = [f"c-{n}-{i}", f"c1-{n}-{i}", f"c2-{n}-{i}", f"c3-{n}-{i}"]
        elif base_type == 'rc':
            candidates = [f"rc-{n}-{i}", f"rc1-{n}-{i}", f"rc2-{n}-{i}", f"rc3-{n}-{i}"]
        else:
            candidates = [f"{base_type}-{n}-{i}"]

        for name in candidates:
            path = instance_dir / f"{name}.inst"
            if path.exists():
                return name, path

        raise FileNotFoundError(
            f"No instance file found in {instance_dir} for candidates: {', '.join(candidates)}"
        )

    # 根据初始算例数据和参数设定，通过计算TSP模型得到T， 输出最终的算例参数

    L_ratios = [0.5, 0.75, 1, 1.25] # 最大完工时间按TSP最优解的倍数

    ns = [50, 100, 150]
    instance_ids = range(1, 6) # 同一规模下算例的标识

    params = {
        "name": [],
        "L_ratio": [],
        "L": []
    }
    for type in types:
        if type == 'r':
            dir = data_root / "random"
        elif type == 'c':
            dir = data_root / "clustered"
        elif type == 'rc':
            dir = data_root / "mixed"

        for n in ns:
            for i in instance_ids:
                for L_ratio in L_ratios:
                    name, instance_path = resolve_instance_path(dir, type, n, i)
                    coords, p = parse_instance_file(str(instance_path))
                    N = list(coords.keys())  
                    para = get_L(coords, N, L_ratio)

                    # 记录所有参数和结果
                    params["name"].append(name)
                    params["L_ratio"].append(L_ratio)
                    params["L"].append(para.L)

                    print(f"{name}\t{L_ratio}\t{para.L}")




# df = pd.DataFrame(params)
# print(df.to_string(index=False))


import math

import itertools

from Utils.TSPmodel import TSP
from Utils.Utils import parse_instance_file

import pandas as pd

class prepare_parameters:

    def __init__(self, coords, alpha, N, m, p, beta ,L_ratio , T_ratio):
        # 输入参数
        self.coords = coords

        self.alpha = alpha

        self.N = N

        self.m = m

        self.p = p

        self.beta = beta

        self.L_ratio = L_ratio

        self.L = 0

        self.T_ratio = T_ratio

        # 输出参数
        self.truck_times, self.drone_times_two, self.euclidean_times = self.calculate_travel_times() # 距离矩阵字典{(i,j)=t_{ij}}

        self.A=self.truck_times.keys()

        self.H=self.build_sorties()

        self.D=self.build_D()

        self.q=self.build_q()

        self.drone_times=self.build_drone_times() # 按sortie集合H中构建sortie的飞行时间, 例如(0, 1, 2):13,...

        self.M=self.build_M()

        self.AVG=self.calculate_AVG()

        self.T=self.build_T()

        self.feasible_drone_times=self.build_feasible_drone_times()



    def calculate_travel_times(self):

        nodes = list(self.coords.keys())

        truck_times = {}

        drone_times = {}

        euclidean_times = {}

        # 遍历所有节点对 (i, j)

        for i in nodes:

            for j in nodes:

                if i == j:

                    continue



                # 获取坐标

                Xi, Yi = self.coords[i]

                Xj, Yj = self.coords[j]



                # t_(i,j) = |X(i) - X(j)| + |Y(i) - Y(j)|

                t_ij = abs(Xi - Xj) + abs(Yi - Yj)

                t_ij = math.floor(t_ij)

                truck_times[(i, j)] = t_ij



                # t'_(i,j) = sqrt((X(i)-X(j))^2 + (Y(i)-Y(j))^2) / alpha

                distance_sq = (Xi - Xj)**2 + (Yi - Yj)**2

                distance = math.sqrt(distance_sq)

                t_prime_ij = math.floor(distance / self.alpha)

                drone_times[(i, j)] = t_prime_ij

                euclidean_times[(i, j)] = distance



        return truck_times, drone_times, euclidean_times



    def build_sorties(self):

        return list(itertools.permutations(self.N, 3))  # 从N中取出3个不同元素排列组合，并生成三元组的列表

        # drone 节点必须不同于发射点和接收点

    def build_D(self):
        """
        根据无人机数量 m，生成一个无人机标识符列表。
        例如，如果 self.m = 4, 则返回 ['d1', 'd2', 'd3', 'd4']。
        """
        # 确保 m 是一个正整数
        if not isinstance(self.m, int) or self.m <= 0:
            return []

        return [f'd{i}' for i in range(1, self.m + 1)]



    def build_q(self):

        # return {h: self.p[h[1]]*self.beta for h in self.H}
        return {h: math.floor(self.p[h[1]] * self.beta) for h in self.H}


    def build_drone_times(self):

        return {h: self.drone_times_two[h[1],h[2]]+self.drone_times_two[h[0],h[1]] for h in self.H}



    def build_M(self):

        M = {}

        for j in self.N:

            vals = [self.drone_times[h] for h in self.H if h[2] == j]

            M[j] = max(vals) if len(vals) > 0 else 0.0

        return M



    def calculate_AVG(self):

        # return sum(self.drone_times_two.values())/len(self.drone_times_two)
        # return sum(self.euclidean_times.values()) / len(self.euclidean_times)
        return sum([math.floor(x) for x in list(self.euclidean_times.values())]) / len(self.euclidean_times)


    def build_feasible_drone_times(self):

        endurance = math.floor(self.AVG*self.L_ratio)

        self.L = endurance

        return {h: self.drone_times[h] for h in self.H if math.floor(self.euclidean_times[(h[0], h[1])]) + math.floor(self.euclidean_times[(h[1], h[2])]) <= endurance}



    def build_T(self):

        tsp=TSP(self.N,self.truck_times)

        tsp.solve()

        t=tsp.optimal_cost

        return math.floor(t*self.T_ratio)


if __name__ == "__main__":
    dir = "./analysis/data/random/" # "./baseline/instances/OP-mD/"
    type = 'r' # poi, r, c, rc

    # 根据初始算例数据和参数设定，通过计算TSP模型得到T， 输出最终的算例参数
    L_ratios = [0.5, 1]
    alphas = [1, 2]  # 无人机的速度的倍数
    betas = [0.67, 1, 1.33] # 无人机采集分数按卡车采集分数的倍数
    T_ratios = [1 / 3, 2 / 3] # 最大完工时间按TSP最优解的倍数
    ms = [1, 2, 3] # 无人机数量取值
    # ns = [10, 20, 30, 40, 50] # 节点数取值
    ns = [75, 100, 125, 150]
    instance_ids = range(1, 6) # 同一规模下算例的标识

    cnt = 1
    total_instances = len(L_ratios) * len(ns) * len(instance_ids) * len(alphas) * len(betas) * len(T_ratios) * len(ms)
    print(f"Total instances: {total_instances}")

    params = {
        "name": [],
        "alpha": [],
        "beta": [],
        "L": [],
        "m": [],
        "T": [],
        "L_ratio": [],
        "T_ratio": [],
    }

    for n in ns:
        for i in instance_ids:
            for alpha in alphas:
                for beta in betas:
                    for L_ratio in L_ratios:
                        for m in ms:
                            for T_ratio in T_ratios:
                                name = f"{type}-{n}-{i}"
                                coords, p = parse_instance_file(dir + name + '.inst')
                                N = list(coords.keys())  # ????????
                                para = prepare_parameters(coords, alpha, N, m, p, beta, L_ratio, T_ratio)
                                inst = {
                                    "N": para.N,
                                    "A": para.A,
                                    "H": para.feasible_drone_times.keys(),
                                    "D": para.D,
                                    "p": para.p,
                                    "q": para.q,
                                    "t": para.truck_times,
                                    "tprime": para.feasible_drone_times,
                                    "T": para.T,
                                    "M": para.M
                                }

                                # 记录所有参数和结果
                                params["L_ratio"].append(L_ratio)
                                params["L"].append(para.L)
                                params["name"].append(name)
                                params["alpha"].append(alpha)
                                params["beta"].append(beta)
                                params["T_ratio"].append(T_ratio)
                                params["T"].append(para.T)
                                params["m"].append(m)
                                if cnt % 10 == 0:
                                    print(
                                        f"{cnt}/{total_instances} instances finished ({cnt / total_instances * 100:.1f}%)")
                                cnt += 1
    desired_columns = [
        # 实例参数
        'name',
        'alpha',
        'beta',
        'L',
        'm',
        'T'
    ]

    df = pd.DataFrame(params, columns=desired_columns)

    # 保存到Excel
    output_file = f'./parameters/instance_parameters_{type}.txt'
    df.to_csv(output_file, sep='\t', index=False, mode='a')



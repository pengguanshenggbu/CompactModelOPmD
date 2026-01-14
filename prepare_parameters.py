import math

import itertools

from TSP import TSP



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

        return {h: self.drone_times[h] for h in self.H if self.euclidean_times[(h[0],h[1])] + self.euclidean_times[(h[1],h[2])] <= endurance}



    def build_T(self):

        tsp=TSP(self.N,self.truck_times)

        tsp.solve()

        t=tsp.optimal_cost

        return math.floor(t*self.T_ratio)
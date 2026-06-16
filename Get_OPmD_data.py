import math
from Utils.Utils import calculate_travel_times
import itertools

class get_OPmD_data:

    def __init__(self, coords, p, alpha, beta, L, m):
        # 输入参数
        self.coords = coords

        self.alpha = alpha

        self.N = list(coords.keys())

        self.m = m

        self.p = p

        self.beta = beta

        self.L = L

        # 输出参数
        self.truck_times, self.drone_times_two, self.euclidean_times = calculate_travel_times(self.coords, self.alpha)  # 距离矩阵字典{(i,j)=t_{ij}}

        self.A=self.truck_times.keys()

        self.H=self.build_sorties()

        self.D=self.build_D()

        self.q=self.build_drone_prize()

        self.sortie_times=self.build_drone_times() # 按sortie集合H中构建sortie的飞行时间, 例如(0, 1, 2):13,...

        self.M=self.build_M()

        self.AVG=self.calculate_AVG()

        self.feasible_sortie_times=self.build_feasible_drone_times()

    
    def build_sorties(self):

        return list(itertools.permutations(self.N, 3))  # 从N中取出3个不同元素排列组合，并生成三元组的列表

        # drone 节点必须不同于发射点和接收点

    def build_D(self):
        """
        根据无人机数量 m 生成一个无人机标识符列表。
        例如，如果 self.m = 4, 则返回 ['d1', 'd2', 'd3', 'd4']。
        """
        # 确保 m 是一个正整数
        if not isinstance(self.m, int) or self.m <= 0:
            return []

        return [f'd{i}' for i in range(1, self.m + 1)]


    def build_drone_prize(self):

        return {h: math.floor(self.p[h[1]] * self.beta) for h in self.H}


    def build_drone_times(self):

        return {h: self.drone_times_two[h[0],h[1]] + self.drone_times_two[h[1],h[2]] for h in self.H}


    def build_M(self):

        M = {}

        for j in self.N:

            vals = [self.sortie_times[h] for h in self.H if h[2] == j]

            M[j] = max(vals) if len(vals) > 0 else 0.0

        return M


    def calculate_AVG(self):

        return sum([math.floor(x) for x in list(self.euclidean_times.values())]) / len(self.euclidean_times)


    def build_feasible_drone_times(self):

        return {h: self.sortie_times[h] for h in self.H if math.floor(self.euclidean_times[(h[0], h[1])]) + math.floor(self.euclidean_times[(h[1], h[2])]) <= self.L}


if __name__ == "__main__":

    from Utils.Utils import parse_instance_file
    import argparse
        # 定义命令行参数解析
    parser = argparse.ArgumentParser(description="输出OPmD模型输入数据",
                                     # 让帮助信息更易读
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # 添加参数，设置默认值以便直接运行也能跑
    parser.add_argument('name', type=str, default=10, help='算例名称')
    parser.add_argument('alpha', type=float, default=1.0, help='Alpha值')
    parser.add_argument('beta', type=float, default=1.0, help='Beta值')
    parser.add_argument('L', type=float, default=15, help='sortie最大长度')
    parser.add_argument('m', type=int, default=1, help='无人机数量')
    parser.add_argument('Tmax', type=float, default=59, help='最大完工时间')
    args = parser.parse_args()

        # 使用命令行传入的参数
    name = args.name
    m = args.m
    alpha = args.alpha
    beta = args.beta
    L = args.L
    Tmax = args.Tmax

    # 读取算例数据
    dir = "./Data/random/"
    coords, p = parse_instance_file(dir + name + '.inst')
    N = list(coords.keys())

    # 根据设定参数和数据，输出求解所需的输入数据
    para = get_OPmD_data(coords, p, alpha, beta, L, m)
    inst = {
        "N": N,
        "A": para.A,
        "H": para.feasible_sortie_times.keys(),
        "D": para.D,
        "p": para.p,
        "q": para.q,
        "t": para.truck_times,
        "tprime": para.feasible_sortie_times,
        "Tmax": Tmax,
        "M": para.M,
        "m": m
    }

    # print truck times
    # print("Truck travel times matrix:")
    # for i in N:
    #     row = []
    #     for j in N:
    #         if i == j:
    #             row.append("0")
    #         else:
    #             row.append(str(para.truck_times[(i, j)]))
    #     print("\t".join(row))
    
    # print drone times
    # print("\nDrone times:")
    # for i in N:
    #     row = []
    #     for j in N:
    #         if i == j:
    #             row.append("0")
    #         else:
    #             row.append(str(para.drone_times_two[(i, j)]))
    #     print("\t".join(row))
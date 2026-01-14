
from InstanceSolver import InstanceSolver
from prepare_parameters import prepare_parameters
import pandas as pd
from pprint import pprint

def parse_instance_file(filename):
    coords = {}  # {i: (X, Y)}
    p = {}  # {i: prize}
    dir = "./baseline/instances/OP-mD/"

    # 2. 获取当前脚本(main.py)所在的目录
    # script_dir = os.path.dirname(os.path.abspath(__file__))

    with open(dir + filename, 'r') as f:
        lines = f.readlines()

    # 第一行是节点数 N
    N_size = int(lines[0].strip())
    N = list(range(N_size))  # 节点集合 N = {0, 1, ..., N-1}

    # 解析节点数据
    for i in N:
        parts = lines[i + 1].strip().split()
        X, Y, prize = float(parts[0]), float(parts[1]), float(parts[2])
        coords[i] = (X, Y)
        p[i] = prize
    return coords, p


if __name__ == "__main__":
    # L_ratios = [0.25, 0.5, 0.75, 1, 1.25]
    # alphas = [1, 2]  # 无人机的速度的倍数
    # betas = [0.67, 1, 1.33] # 无人机采集分数按卡车采集分数的倍数
    # T_ratios = [1 / 3, 2 / 3] # 最大完工时间按TSP最优解的倍数
    # ms = [1, 2, 3] # 节点数取值
    # ns = [10, 20] # 无人机数量取值
    L_ratios = [0.5, 1]
    alphas = [1, 2]
    betas = [0.67, 1, 1.33]
    T_ratios = [1 / 3, 2 / 3]
    ms = [1, 2, 3]
    ns = [10, 20, 30, 40, 50]
    instance_ids = range(1, 6)

    # L = []

    # 初始化结果字典，包含所有参数
    results = {
        "L_ratio": [],
        "L": [],
        "n": [],
        "instance_name": [],
        "alpha": [],
        "beta": [],
        "T_ratio": [],
        "T": [],
        "m": [],
        "CPU": [],
        "SolCount": [],
        "nodes": [],
        "status": [],
        "OBJ": [],
        "GAP": [],
        "obj_bound": []
    }

    cnt = 1
    total_instances = len(L_ratios) * len(ns) * 5 * len(alphas) * len(betas) * len(T_ratios) * len(ms)
    print(f"Total instances to solve: {total_instances}")

    for n in ns:
        for i in instance_ids:
            for alpha in alphas:
                for beta in betas:
                    for L_ratio in L_ratios:
                        for m in ms:
                            for T_ratio in T_ratios:
                                try:
                                    name = f"poi-{n}-{i}"
                                    coords, p = parse_instance_file(name+'.inst')
                                    N = list(coords.keys()) # 点索引集合
                                    para = prepare_parameters(coords, alpha, N, m, p, beta, L_ratio, T_ratio)

                                    # 记录所有参数和结果
                                    results["L_ratio"].append(L_ratio)
                                    results["L"].append(para.L)
                                    results["n"].append(n)
                                    results["instance_name"].append(name)
                                    results["alpha"].append(alpha)
                                    results["beta"].append(beta)
                                    results["T_ratio"].append(T_ratio)
                                    results["T"].append(para.T)
                                    results["m"].append(m)
                                    results["i"].append(i)

                                    if cnt % 10 == 0:
                                        print(
                                            f"{cnt}/{total_instances} instances finished ({cnt / total_instances * 100:.1f}%)")
                                    cnt += 1

                                except Exception as e:
                                    print(f"Error at instance {cnt}: {e}")
                                    print(
                                        f"Parameters: n={n}, i={i}, alpha={alpha}, beta={beta}, T_ratio={T_ratio}, m={m}")
                                    cnt += 1
                                    continue

    # pprint(results)

    # 1. 定义你想要的列名和顺序
    desired_columns = [
        # 实例参数
        'instance_name',
        'alpha',
        'beta',
        'L',
        'm',
        'T',
        'n',
        'i'
    ]
    # 保存最终结果
    # df = pd.DataFrame(results)
    df = pd.DataFrame(results, columns=desired_columns)

    # 保存到Excel
    output_file = f'Instance_parameters.txt'
    df.to_csv(output_file, sep='\t', index=False)

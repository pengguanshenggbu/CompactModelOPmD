
from InstanceSolver import InstanceSolver
from prepare_parameters import prepare_parameters
import pandas as pd
from pprint import pprint
from OPmDPlot import plot_truck_drone_route

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


def print_full_route(edges, start_node=0):
    """
    根据边集合重建并打印完整路线。
    :param edges: 边列表，例如 [(0, 1), (1, 2), (2, 0)]
    :param start_node: 初始节点，默认为 0 (Depot)
    """
    if not edges:
        print("Truck Route: Empty")
        return

    # 1. 构建邻接字典 {起点: 终点}
    # 假设每个点只有一个后继（TSP/VRP特性）
    adj = {u: v for u, v in edges}

    # 2. 从初始点开始遍历
    curr = start_node
    route = [curr]

    # 设置最大循环次数防止死循环（以防数据异常）
    max_steps = len(edges) + 1
    steps = 0

    while curr in adj and steps < max_steps:
        nxt = adj[curr]
        route.append(nxt)
        curr = nxt
        steps += 1

        # 如果回到起点，说明闭环，结束
        if curr == start_node:
            break

    # 3. 打印结果
    route_str = " -> ".join(map(str, route))
    print(f"Truck Route: {route_str}")
def print_drone_routes(drone_data):
    """
    打印无人机的飞行路线方案。
    :param drone_data: 列表，结构为 [((launch, visit, land), drone_id), ...]
                       例如: [((0, 5, 2), 'd1'), ((2, 6, 3), 'd2')]
    """
    if not drone_data:
        print("Drone Routes: None")
        return

    # 1. 按照无人机ID分组
    drone_sorties = {}
    for (launch, visit, land), drone_id in drone_data:
        if drone_id not in drone_sorties:
            drone_sorties[drone_id] = []
        drone_sorties[drone_id].append((launch, visit, land))

    # 2. 排序并打印每架无人机的架次
    # 尝试对ID进行排序，如果ID是字符串且包含数字(如'd1'), 直接排序可能按字典序
    try:
        sorted_drones = sorted(drone_sorties.keys())
    except:
        sorted_drones = drone_sorties.keys()

    for d_id in sorted_drones:
        sorties = drone_sorties[d_id]
        # 格式化为 "[1->5->2]" 的形式
        routes_str = ", ".join([f"[{i}->{k}->{j}]" for i, k, j in sorties])
        print(f"Drone {d_id} Route: {routes_str}")


def calculate_arrival_times(truck_edges, drone_sorties, truck_times, drone_times, W_vals, start_node=0):
    """
    计算并打印卡车和无人机到达每个节点的时间。
    """
    if not truck_edges:
        return

    # --- 1. 重建卡车有序路径 ---
    adj = {u: v for u, v in truck_edges}
    curr = start_node
    truck_route_ordered = [curr]
    while curr in adj:
        nxt = adj[curr]
        truck_route_ordered.append(nxt)
        curr = nxt
        if curr == start_node: break

    # --- 2. 计算卡车到达时间 ---
    # ArrivalTime[i] = ArrivalTime[prev] + WaitTime[prev] + TravelTime[prev->i]
    truck_arrival_times = {start_node: 0.0}
    current_time = 0.0

    print("\n--- Node Arrival Times ---")
    print(f"Node {start_node} (Depot): Arrival {current_time:.2f}, Wait {W_vals.get(start_node, 0):.2f}")

    for i in range(len(truck_route_ordered) - 1):
        u = truck_route_ordered[i]
        v = truck_route_ordered[i + 1]

        wait_u = W_vals.get(u, 0.0)
        travel_uv = truck_times.get((u, v), 0)

        # 更新时间：当前到达时间 + 在u的等待 + 行驶到v的时间
        current_time += wait_u + travel_uv
        truck_arrival_times[v] = current_time

        print(f"Node {v}: Arrival {current_time:.2f} (Truck) | Prev Wait: {wait_u:.2f}, Travel: {travel_uv}")

    # --- 3. 计算无人机到达时间 ---
    # 无人机从 i 发射，到达 k。发射时间通常视为卡车到达 i 的时间（或卡车离开 i 的时间，模型中通常假设并发）
    # 这里假设无人机在卡车到达 i 并准备出发时发射
    if drone_sorties:
        print("\n--- Drone Customer Arrival Times ---")
        # drone_sorties 结构可能是 [((i, k, j), d_id), ...] 或 [(i, k, j, d_idx)...] 取决于之前的处理
        # 这里适配 main.py 最后的处理逻辑

        for item in drone_sorties:
            # 兼容不同的数据结构
            if len(item) == 2 and isinstance(item[0], tuple):
                (i, k, j), d_id = item
            elif len(item) == 4:
                i, k, j, d_id = item
            else:
                continue

            launch_time = truck_arrival_times.get(i, 0.0)

            # 获取无人机飞行时间 t'(i, k)
            # 注意：results["drone_times"] 存储的是 (u,v)->time 的字典
            t_ik = drone_times.get((i, k), 0)

            arrival_k = launch_time + t_ik

            print(f"Node {k} (Drone {d_id}): Arrival {arrival_k:.2f} [Launched from {i} at {launch_time:.2f}]")
# --- 使用示例 (放在你的 main.py 最后的循环中) ---
# for plot_index in range(0, len(results["instance_name"])):
#     truck_edges = results["truckRoute"][plot_index]
#
#     print(f"\nInstance: {results['instance_name'][plot_index]}")
#     print_full_route(truck_edges)  # <--- 调用函数
#
#     # ... (其余绘图代码)

# L_ratio=0.25
# N=10
# i=2
# alpha=1
# beta=1
# T_ratio=0.67
# m=3
# coords,p=parse_instance_file(f"poi-10-{i}.inst")
# N=list(coords.keys())
# para=prepare_parameters(coords,alpha,N,m,p,beta,L_ratio,T_ratio)
# inst={"N":para.N,"A":para.A,"H":para.feasible_drone_times.keys(),"D":para.D,"p":para.p,"q":para.q,"t":para.truck_times,"tprime":para.feasible_drone_times,"T":para.T,"M":para.M}
# instance=InstanceSolver(inst, 1)
# result=instance.solve()
# result

if __name__ == "__main__":
    # L_ratios = [0.5, 1]
    # alphas = [1, 2]  # 无人机的速度的倍数
    # betas = [0.67, 1, 1.33] # 无人机采集分数按卡车采集分数的倍数
    # T_ratios = [1 / 3, 2 / 3] # 最大完工时间按TSP最优解的倍数
    # ms = [1, 2, 3] # 无人机数量取值
    # ns = [10, 20, 30, 40, 50] # 节点数取值
    L_ratios = [1]
    alphas = [2]  # 无人机的速度的倍数
    betas = [0.67] # 无人机采集分数按卡车采集分数的倍数
    T_ratios = [1 / 3] # 最大完工时间按TSP最优解的倍数
    ms = [1] # 无人机数量取值
    ns = [10] # 节点数取值
    instance_ids = range(1, 2)

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
        "obj_bound": [],
        # 路线方案
        "truckRoute":[],
        "droneRoute":[],
        "coords":[],
        "D": [],
        "truck_times":[],
        "drone_times":[],
        "p":[],
        "q": [],
        "Wait":[]
    }

    cnt = 1
    total_instances = len(L_ratios) * len(ns) * len(instance_ids) * len(alphas) * len(betas) * len(T_ratios) * len(ms)
    print(f"Total instances to solve: {total_instances}")

    for n in ns:
        for i in instance_ids:
            for alpha in alphas:
                for beta in betas:
                    for L_ratio in L_ratios:
                        for m in ms:
                            for T_ratio in T_ratios:
                                # try:
                                    name = f"poi-{n}-{i}"
                                    coords, p = parse_instance_file(name+'.inst')
                                    N = list(coords.keys()) # 点索引集合
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
                                    instance = InstanceSolver(inst, 1)
                                    result = instance.solve()

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
                                    results["status"].append(result["status"])

                                    results["CPU"].append(result["Runtime"])
                                    # results["SolCount"].append(result["SolCount"])
                                    results["nodes"].append(result["NodeCount"])
                                    results["OBJ"].append(result["ObjVal"])
                                    results["GAP"].append(result["GAP"])
                                    results["obj_bound"].append(result["obj_bound"])
                                    results["truckRoute"].append(result["truckRoute"])
                                    results["droneRoute"].append(result["droneRoute"])


                                    results["D"].append(para.D)
                                    results["coords"].append(coords)
                                    results["truck_times"].append(para.truck_times)
                                    results["drone_times"].append(para.drone_times_two)
                                    results["p"].append(para.p)
                                    results["q"].append(para.q)
                                    # results["Wait"].append(result["Wait"])

                                    # if result["status"] == 2:
                                    #     results["Runtime"].append(result["Runtime"])
                                    #     results["SolCount"].append(result["SolCount"])
                                    #     results["NodeCount"].append(result["NodeCount"])
                                    #     results["ObjVal"].append(result["ObjVal"])
                                    # else:
                                    #     results["Runtime"].append(None)
                                    #     results["SolCount"].append(None)
                                    #     results["NodeCount"].append(None)
                                    #     print(
                                    #         f"Warning: Instance failed - i={i}, alpha={alpha}, beta={beta}, T_ratio={T_ratio}, m={m}")
                                    #     print(f"Status: {result}")

                                    if cnt % 10 == 0:
                                        print(
                                            f"{cnt}/{total_instances} instances finished ({cnt / total_instances * 100:.1f}%)")
                                    cnt += 1

                                # except Exception as e:
                                #     print(f"Error at instance {cnt}: {e}")
                                #     print(
                                #         f"Parameters: n={n}, i={i}, alpha={alpha}, beta={beta}, T_ratio={T_ratio}, m={m}")
                                #     cnt += 1
                                #     continue

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
        # 求解结果
        'status',
        'OBJ',
        "GAP",
        'CPU',
        'nodes',
        "obj_bound"
    ]
    # 保存最终结果
    df = pd.DataFrame(results, columns=desired_columns)

    # 保存到Excel
    output_file = f'MIPResults_complete.xlsx'
    df.to_excel(output_file, index=False)
    # print("\nDataFrame with selected columns and order:")
    pd.set_option('display.max_columns', None)
    #
    # # 2. 设置显示所有行
    pd.set_option('display.max_rows', None)
    print(df)



    # 画出路线图
    # plot_index = 0
    # for plot_index in range(0, len(results["instance_name"])):
    #     truck_edges = results["truckRoute"][plot_index]
    #     print_full_route(truck_edges)
    #     drone_sorties =[(h[0], h[1], h[2], results["D"][plot_index].index(d)) for (h, d) in results["droneRoute"][plot_index]]
    #     print_drone_routes(results["droneRoute"][plot_index])
    #     coords = list(results["coords"][plot_index].values())
    #     truckDis = results["truck_times"][plot_index]
    #     droneDis = results["drone_times"][plot_index]
    #     instanceName = results["instance_name"][plot_index]
    #     truckPrize = results["p"][plot_index]
    #     dronePrize = results["q"][plot_index]
    #     Tmax = results["T"][plot_index]
    #     L = results["L"][plot_index]
    #     instanceData = {"instanceName": instanceName,"coordinates":coords,"truckPrize": truckPrize, "dronePrize":dronePrize,
    #                     "truckDistance": truckDis, "droneDistance": droneDis, "Tmax": Tmax, "L": L}
        # calculate_arrival_times(
        #     truck_edges,
        #     results["droneRoute"][plot_index],
        #     truckDis,
        #     droneDis,
        #     results["Wait"][plot_index]
        # )
        # plot_truck_drone_route(truck_edges, drone_sorties, instanceData, save_path=f"./figure/{instanceName}.png")
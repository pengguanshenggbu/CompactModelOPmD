import math

def parse_instance_file(fileDir):
    # 读取算例节点数据;输出节点坐标和奖励字典
    coords = {}  # {i: (X, Y)}
    p = {}  # {i: prize}
    # dir = "./Data/baseline/instances/OP-mD/"

    # 2. 获取当前脚本(main.py)所在的目录
    # script_dir = os.path.dirname(os.path.abspath(__file__))

    with open(fileDir, 'r') as f:
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

def calculate_travel_times(coords, alpha):
    # 根据坐标数据计算OPmD的旅行时间矩阵, alpha为drone相对truck的速度比值；输出卡车-无人机的旅行时间矩阵字典

    nodes = list(coords.keys())

    truck_times = {}

    drone_times = {}

    euclidean_times = {}

    # 遍历所有节点对 (i, j)

    for i in nodes:

        for j in nodes:

            if i == j:

                continue

            # 获取坐标

            Xi, Yi = coords[i]

            Xj, Yj = coords[j]

            # truck travel time: t_(i,j) = |X(i) - X(j)| + |Y(i) - Y(j)|

            truck_times[(i, j)] = math.floor(abs(Xi - Xj) + abs(Yi - Yj))

            # drone travel time: t'_(i,j) = sqrt((X(i)-X(j))^2 + (Y(i)-Y(j))^2) / alpha

            distance = math.sqrt((Xi - Xj)**2 + (Yi - Yj)**2)

            drone_times[(i, j)] = math.floor(distance / alpha)

            euclidean_times[(i, j)] = distance # 非整数

    return truck_times, drone_times, euclidean_times

def calculate_AVG(euclidean_times):
    # 给定两点之间的欧式距离矩阵，输出平均欧式距离AVG

    return sum([math.floor(x) for x in list(euclidean_times.values())]) / len(euclidean_times)

def calculate_endurance(AVG, L_ratio):
    endurance = math.floor(AVG * L_ratio)

    return endurance


def print_truck_route(edges, start_node=0):
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


def save_result_txt(output_str, output_dir, filename ):
    # Construct the filename
    file_path = os.path.join(output_dir, filename)
    with open(file_path, 'a', encoding='utf-8') as f:
        f.write(output_str+"\n")
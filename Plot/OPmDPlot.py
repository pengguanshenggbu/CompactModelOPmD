import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np


def plot_truck_drone_route(truck_edges, drone_sorties, instanceData, save_path='truck_drone_route.png'):
    """
    卡车与多无人机协同配送路线可视化 (修改版)
    - 节点统一绘制，不区分访问类型
    - 坐标轴固定 0-50
    """
    coords_list = instanceData["coordinates"]
    instanceName = instanceData["instanceName"]
    truckDist = instanceData.get("truckDistance", None)
    truckPrize = instanceData.get("truckPrize", None)
    # dronePrize = instanceData.get("dronePrize", None)
    # droneDis = instanceData.get("droneDistance", None)
    Tmax = instanceData["Tmax"]
    L = instanceData["L"]

    N = len(coords_list)

    # --- 1. 数据预处理 ---
    # 处理无人机数据，标准化为 (i, k, j, d) 以便绘制路线
    normalized_sorties = []
    active_drones = set()

    for item in drone_sorties:
        if len(item) == 4:
            i, k, j, d = item
        else:
            i, k, j = item
            d = 0  # 默认 ID

        normalized_sorties.append((i, k, j, d))
        active_drones.add(d)

    # 寻找起点 (Depot) - 通常是 0
    start_node = 0

    # --- 2. 绘图设置 ---
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(10, 10))  # 调整为正方形画布，配合固定坐标轴

    # 颜色定义
    COLOR_TRUCK = '#1f77b4'  # 深蓝
    COLOR_DEPOT = '#FFD700'  # 金色

    # 定义无人机颜色调色板
    DRONE_PALETTE = [
        '#d62728', '#2ca02c', '#ff7f0e', '#9467bd',
        '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf'
    ]

    def get_drone_color(d_id):
        return DRONE_PALETTE[d_id % len(DRONE_PALETTE)]

    # --- 3. 绘制卡车路线 (实线) ---
    ax.plot([], [], color=COLOR_TRUCK, linewidth=2.5, label='Truck Route')

    for u, v in truck_edges:
        x1, y1 = coords_list[u]
        x2, y2 = coords_list[v]
        ax.plot([x1, x2], [y1, y2], color=COLOR_TRUCK, linewidth=2.5, alpha=0.7, zorder=1)

        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.annotate('', xy=(mx, my), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='-|>', color=COLOR_TRUCK, lw=2, mutation_scale=15),
                    zorder=1)

    # --- 4. 绘制无人机路线 (弧线虚线) ---
    sorted_drones = sorted(list(active_drones))
    for d in sorted_drones:
        c = get_drone_color(d)
        ax.plot([], [], color=c, linestyle='--', linewidth=1.5, label=f'Drone {d} Route')

    for i, k, j, d in normalized_sorties:
        color = get_drone_color(d)
        xi, yi = coords_list[i]
        xk, yk = coords_list[k]
        xj, yj = coords_list[j]

        # Launch: i -> k
        ax.annotate('', xy=(xk, yk), xytext=(xi, yi),
                    arrowprops=dict(arrowstyle='-|>', color=color, linestyle='--', lw=1.5,
                                    mutation_scale=15, connectionstyle="arc3,rad=0.2"),
                    zorder=10)

        # Land: k -> j
        ax.annotate('', xy=(xj, yj), xytext=(xk, yk),
                    arrowprops=dict(arrowstyle='-|>', color=color, linestyle='--', lw=1.5,
                                    mutation_scale=15, connectionstyle="arc3,rad=0.2"),
                    zorder=10)

    # --- 5. 绘制节点 (修改部分) ---

    # A. 绘制 Depot (保持特殊样式)
    dx, dy = coords_list[start_node]
    ax.scatter(dx, dy, c=COLOR_DEPOT, edgecolors='black', marker='*', s=800, zorder=10, label='Depot')

    # B. 绘制所有其他节点 (统一样式，不区分卡车/无人机/未访问)
    other_xs = []
    other_ys = []
    for idx, (x, y) in enumerate(coords_list):
        if idx == start_node:
            continue
        other_xs.append(x)
        other_ys.append(y)

    # 统一绘制为白色填充、黑色边框的圆形
    ax.scatter(other_xs, other_ys, c='white', edgecolors='black', marker='o', s=300, linewidth=1.5, zorder=5,
               label='Customer Node')

    # --- 6. 添加标签 ---
    for idx, (x, y) in enumerate(coords_list):
        ax.annotate(str(idx), xy=(x, y), xytext=(0, 0), textcoords='offset points',
                    fontsize=12, fontweight='bold', color='black', ha='center', va='center', zorder=15)

    # --- 7. 图表装饰 (修改部分) ---
    title_text = f"OP-mD {instanceName}"
    if truckDist:
        t_dist = sum(truckDist.get((u, v), 0) for u, v in truck_edges)
        # d_dist = sum(truckDist.get((i, k), 0) + truckDist.get((k, j), 0) for i, k, j, d in normalized_sorties)
        title_text += f"\nTruck Dist: {t_dist}/{Tmax} L:{L}"
    if truckPrize:
        t_prize = sum(truckPrize.get((v), 0) for u, v in truck_edges)
        title_text += f"\nTruck Prize: {t_prize}"

    ax.set_title(title_text, fontsize=15, fontweight='bold', pad=20)
    # ax.set_xlabel('X Coordinate')
    # ax.set_ylabel('Y Coordinate')

    # 设置固定坐标轴范围
    ax.set_xlim(0, 55)
    ax.set_ylim(0, 55)
    ax.set_aspect('equal')  # 保持比例一致

    # 去除边框
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # 图例
    # plt.legend(loc='upper right', frameon=True, fancybox=True, framealpha=0.9, title="Legend")

    plt.tight_layout()
    # plt.savefig(save_path, dpi=300, bbox_inches='tight')
    # print(f"图片已保存至: {save_path}")
    plt.show()


# --- 测试代码 ---
if __name__ == "__main__":
    # 1. 定义坐标 (0是仓库)
    # 0-50 范围内的坐标样例 (N=15)
    coords = [
        (25, 25),  # 0: Depot (中心)
        (5, 10),   # 1
        (12, 45),  # 2
        (48, 48),  # 3
        (45, 5),   # 4
        (2, 30),   # 5
        (15, 15),  # 6
        (35, 35),  # 7
        (40, 10),  # 8
        (10, 40),  # 9
        (20, 5),   # 10
        (5, 48),   # 11
        (30, 2),   # 12
        (50, 25),  # 13
        (25, 50)   # 14
    ]

    # 2. 定义卡车路径 (0 -> 1 -> 2 -> 3 -> 4 -> 0)
    truck_edges = [
        (0, 1), (1, 2), (2, 3), (3, 4), (4, 0)
    ]

    # 3. 定义无人机架次 (Launch, Visit, Land, Drone_ID)
    # 无人机 0 (红色) 负责左侧和右下
    # 无人机 1 (绿色) 负责右侧和左下
    drone_sorties = [
        (1, 5, 2, 0),  # Drone 0: 从 1 飞去 5，在 2 降落
        (2, 6, 3, 1),  # Drone 1: 从 2 飞去 6，在 3 降落
        (3, 7, 4, 0),  # Drone 0: 从 3 飞去 7，在 4 降落
        (4, 8, 0, 1)  # Drone 1: 从 4 飞去 8，在 0 降落
    ]

    # 简单的距离矩阵用于显示
    import math

    dist_matrix = {}
    for i in range(len(coords)):
        for j in range(len(coords)):
            d = math.floor(math.sqrt((coords[i][0] - coords[j][0]) ** 2 + (coords[i][1] - coords[j][1]) ** 2))
            dist_matrix[i, j] = d

    plot_truck_drone_route(truck_edges, drone_sorties, coords,'test')

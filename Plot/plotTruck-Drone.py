import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as pe

# 统一设置中文字体，确保美观且不乱码
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

# 统一的颜色美化配置
COLOR_NODE = '#3498db'  # 节点蓝色
COLOR_TRUCK_PATH = '#95a5a6'  # 卡车路径灰色
COLOR_DRONE_PATH = '#e74c3c'  # 无人机路径红色
COLOR_TRUCK = '#2ecc71'  # 卡车绿色
COLOR_STATION = '#9b59b6'  # 固定站点紫色


def plot_synchronous_mode(ax):
    """
    绘制卡车-无人机同步模式示意图。
    无人机从卡车起飞，执行任务后返回到同一辆卡车。
    """
    ax.set_title("(a) 同步模式 (Synchronous Mode)", fontsize=14, fontweight='bold', pad=10)
    ax.set_xlim(-1, 10)
    ax.set_ylim(-1, 6)
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')  # 隐藏坐标轴

    # 定义关键节点坐标
    depot = (0.5, 3)
    node1 = (2.5, 3)
    node2_launch = (4.5, 3)  # 无人机起飞点，卡车在此等待
    customer_a = (6.5, 5)  # 客户点
    node2_land = (4.5, 3)  # 无人机降落点，与起飞点相同
    node3 = (7.5, 3)
    depot_end = (9.5, 3)

    nodes_coords = {
        "Depot": depot,
        "Node 1": node1,
        "Node 2 (起飞/降落)": node2_launch,
        "客户 A": customer_a,
        "Node 3": node3,
        "Depot (终点)": depot_end
    }

    # 绘制节点
    for name, (x, y) in nodes_coords.items():
        ax.plot(x, y, 'o', markersize=12, color=COLOR_NODE, markeredgecolor='white', markeredgewidth=1.5, zorder=5)
        ax.text(x, y + 0.35, name, ha='center', va='bottom', fontsize=10,
                path_effects=[pe.Stroke(linewidth=1, foreground='white'), pe.Normal()])

    # 绘制卡车路径
    truck_path_coords = [depot, node1, node2_launch, node3, depot_end]
    ax.plot([p[0] for p in truck_path_coords], [p[1] for p in truck_path_coords],
            '--', color=COLOR_TRUCK_PATH, linewidth=2.5, label='卡车路径', zorder=1)

    # 绘制卡车图标 (在Node 2等待)
    truck_icon_x, truck_icon_y = node2_launch
    ax.add_patch(patches.Rectangle((truck_icon_x - 0.3, truck_icon_y - 0.2), 0.6, 0.4,
                                   facecolor=COLOR_TRUCK, edgecolor='white', linewidth=1.5, zorder=3))
    ax.text(truck_icon_x, truck_icon_y, 'T', color='white', ha='center', va='center', fontsize=10,
            fontweight='bold')

    # 绘制无人机路径 (起飞 -> 客户 -> 降落，使用弧线使其更像飞行轨迹)
    drone_launch_point = node2_launch
    drone_land_point = node2_land
    ax.annotate('', xy=customer_a, xytext=drone_launch_point,
                arrowprops=dict(facecolor=COLOR_DRONE_PATH, edgecolor=COLOR_DRONE_PATH,
                                shrink=0.08, width=1.5, headwidth=8, headlength=10, connectionstyle="arc3,rad=-0.15"),
                zorder=2)
    ax.annotate('', xy=drone_land_point, xytext=customer_a,
                arrowprops=dict(facecolor=COLOR_DRONE_PATH, edgecolor=COLOR_DRONE_PATH,
                                shrink=0.08, width=1.5, headwidth=8, headlength=10, connectionstyle="arc3,rad=-0.15"),
                zorder=2)

    # 绘制无人机图标 (在客户点执行任务)
    drone_icon_x, drone_icon_y = customer_a
    ax.plot(drone_icon_x, drone_icon_y, '^', markersize=10, color=COLOR_DRONE_PATH, zorder=4)
    ax.text(drone_icon_x, drone_icon_y + 0.35, 'D', ha='center', va='bottom', fontsize=10, color=COLOR_DRONE_PATH,
            fontweight='bold',
            path_effects=[pe.Stroke(linewidth=1, foreground='white'), pe.Normal()])

    # 添加动作说明
    ax.text(node2_launch[0] + 0.5, node2_launch[1] + 0.6, '无人机起飞', fontsize=9, color=COLOR_DRONE_PATH)
    ax.text(node2_land[0] + 0.5, node2_land[1] - 0.6, '无人机降落', fontsize=9, color=COLOR_DRONE_PATH)
    ax.text(node2_launch[0] + 0.5, node2_launch[1] + 0.15, '卡车等待', fontsize=9, color=COLOR_TRUCK)


def plot_asynchronous_mode(ax):
    """
    绘制卡车-无人机异步模式示意图。
    无人机从卡车起飞，执行任务后返回到卡车路径上的下一个预定汇合点。
    """
    ax.set_title("(b) 异步模式 (Asynchronous Mode)", fontsize=14, fontweight='bold', pad=10)
    ax.set_xlim(-1, 10)
    ax.set_ylim(-1, 6)
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')  # 隐藏坐标轴

    # 定义关键节点坐标
    depot = (0.5, 3)
    node1 = (2.5, 3)
    node2_launch = (4.5, 3)  # 无人机起飞点
    customer_a = (6.5, 5)  # 客户点
    node3_rendezvous = (7.5, 3)  # 无人机降落点，卡车在此等待汇合
    node4 = (9.5, 3)
    depot_end = (9.5, 3)  # 为简化，与Node 4相同

    nodes_coords = {
        "Depot": depot,
        "Node 1": node1,
        "Node 2 (起飞)": node2_launch,
        "客户 A": customer_a,
        "Node 3 (汇合点)": node3_rendezvous,
        "Node 4": node4
    }

    # 绘制节点
    for name, (x, y) in nodes_coords.items():
        ax.plot(x, y, 'o', markersize=12, color=COLOR_NODE, markeredgecolor='white', markeredgewidth=1.5, zorder=5)
        ax.text(x, y + 0.35, name, ha='center', va='bottom', fontsize=10,
                path_effects=[pe.Stroke(linewidth=1, foreground='white'), pe.Normal()])

    # 绘制卡车路径
    truck_path_coords = [depot, node1, node2_launch, node3_rendezvous, node4]
    ax.plot([p[0] for p in truck_path_coords], [p[1] for p in truck_path_coords],
            '--', color=COLOR_TRUCK_PATH, linewidth=2.5, label='卡车路径', zorder=1)

    # 绘制卡车图标 (在Node 2起飞无人机后继续行驶)
    truck_icon_x, truck_icon_y = node2_launch
    ax.add_patch(patches.Rectangle((truck_icon_x - 0.3, truck_icon_y - 0.2), 0.6, 0.4,
                                   facecolor=COLOR_TRUCK, edgecolor='white', linewidth=1.5, zorder=3))
    ax.text(truck_icon_x, truck_icon_y, 'T', color='white', ha='center', va='center', fontsize=10,
            fontweight='bold')

    # 绘制无人机路径 (起飞 -> 客户 -> 汇合点)
    drone_launch_point = node2_launch
    drone_land_point = node3_rendezvous  # 降落到不同的汇合点
    ax.annotate('', xy=customer_a, xytext=drone_launch_point,
                arrowprops=dict(facecolor=COLOR_DRONE_PATH, edgecolor=COLOR_DRONE_PATH,
                                shrink=0.08, width=1.5, headwidth=8, headlength=10, connectionstyle="arc3,rad=-0.15"),
                zorder=2)
    ax.annotate('', xy=drone_land_point, xytext=customer_a,
                arrowprops=dict(facecolor=COLOR_DRONE_PATH, edgecolor=COLOR_DRONE_PATH,
                                shrink=0.08, width=1.5, headwidth=8, headlength=10, connectionstyle="arc3,rad=0.15"),
                zorder=2)

    # 绘制无人机图标 (在客户点执行任务)
    drone_icon_x, drone_icon_y = customer_a
    ax.plot(drone_icon_x, drone_icon_y, '^', markersize=10, color=COLOR_DRONE_PATH, zorder=4)
    ax.text(drone_icon_x, drone_icon_y + 0.35, 'D', ha='center', va='bottom', fontsize=10, color=COLOR_DRONE_PATH,
            fontweight='bold',
            path_effects=[pe.Stroke(linewidth=1, foreground='white'), pe.Normal()])

    # 添加动作说明
    ax.text(node2_launch[0] + 0.5, node2_launch[1] + 0.6, '无人机起飞', fontsize=9, color=COLOR_DRONE_PATH)
    ax.text(node3_rendezvous[0] + 0.5, node3_rendezvous[1] - 0.6, '无人机降落 (汇合点)', fontsize=9,
            color=COLOR_DRONE_PATH)
    ax.text(node2_launch[0] + 0.5, node2_launch[1] + 0.15, '卡车继续行驶', fontsize=9, color=COLOR_TRUCK)


def plot_fixed_station_mode(ax):
    """
    绘制无人机可回收到分散部署的固定站点模式示意图。
    无人机从卡车起飞，执行任务后返回到预设的固定无人机站点。
    """
    ax.set_title("(c) 无人机可回收到分散部署的固定站点", fontsize=14, fontweight='bold', pad=10)
    ax.set_xlim(-1, 10)
    ax.set_ylim(-1, 6)
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')  # 隐藏坐标轴

    # 定义关键节点坐标
    depot = (0.5, 3)
    node1 = (2.5, 3)
    node2_launch = (4.5, 3)  # 无人机起飞点
    customer_a = (6.5, 5)  # 客户点
    fixed_station_x = (8.5, 1)  # 固定无人机站点
    node3 = (7.5, 3)
    depot_end = (9.5, 3)

    nodes_coords = {
        "Depot": depot,
        "Node 1": node1,
        "Node 2 (起飞)": node2_launch,
        "客户 A": customer_a,
        "Node 3": node3,
        "Depot (终点)": depot_end
    }

    # 绘制节点
    for name, (x, y) in nodes_coords.items():
        ax.plot(x, y, 'o', markersize=12, color=COLOR_NODE, markeredgecolor='white', markeredgewidth=1.5, zorder=5)
        ax.text(x, y + 0.35, name, ha='center', va='bottom', fontsize=10,
                path_effects=[pe.Stroke(linewidth=1, foreground='white'), pe.Normal()])

    # 绘制固定无人机站点
    ax.plot(fixed_station_x[0], fixed_station_x[1], 's', markersize=14, color=COLOR_STATION, markeredgecolor='white',
            markeredgewidth=1.5, zorder=5)
    ax.text(fixed_station_x[0], fixed_station_x[1] - 0.5, '固定无人机站点 X', ha='center', va='top', fontsize=10,
            color=COLOR_STATION,
            path_effects=[pe.Stroke(linewidth=1, foreground='white'), pe.Normal()])

    # 绘制卡车路径
    truck_path_coords = [depot, node1, node2_launch, node3, depot_end]
    ax.plot([p[0] for p in truck_path_coords], [p[1] for p in truck_path_coords],
            '--', color=COLOR_TRUCK_PATH, linewidth=2.5, label='卡车路径', zorder=1)

    # 绘制卡车图标 (在Node 2起飞无人机后继续行驶)
    truck_icon_x, truck_icon_y = node2_launch
    ax.add_patch(patches.Rectangle((truck_icon_x - 0.3, truck_icon_y - 0.2), 0.6, 0.4,
                                   facecolor=COLOR_TRUCK, edgecolor='white', linewidth=1.5, zorder=3))
    ax.text(truck_icon_x, truck_icon_y, 'T', color='white', ha='center', va='center', fontsize=10,
            fontweight='bold')

    # 绘制无人机路径 (起飞 -> 客户 -> 固定站点)
    drone_launch_point = node2_launch
    drone_land_point = fixed_station_x  # 降落到固定站点
    ax.annotate('', xy=customer_a, xytext=drone_launch_point,
                arrowprops=dict(facecolor=COLOR_DRONE_PATH, edgecolor=COLOR_DRONE_PATH,
                                shrink=0.08, width=1.5, headwidth=8, headlength=10, connectionstyle="arc3,rad=-0.15"),
                zorder=2)
    ax.annotate('', xy=drone_land_point, xytext=customer_a,
                arrowprops=dict(facecolor=COLOR_DRONE_PATH, edgecolor=COLOR_DRONE_PATH,
                                shrink=0.08, width=1.5, headwidth=8, headlength=10, connectionstyle="arc3,rad=-0.1"),
                zorder=2)

    # 绘制无人机图标 (在客户点执行任务)
    drone_icon_x, drone_icon_y = customer_a
    ax.plot(drone_icon_x, drone_icon_y, '^', markersize=10, color=COLOR_DRONE_PATH, zorder=4)
    ax.text(drone_icon_x, drone_icon_y + 0.35, 'D', ha='center', va='bottom', fontsize=10, color=COLOR_DRONE_PATH,
            fontweight='bold',
            path_effects=[pe.Stroke(linewidth=1, foreground='white'), pe.Normal()])

    # 添加动作说明
    ax.text(node2_launch[0] + 0.5, node2_launch[1] + 0.6, '无人机起飞', fontsize=9, color=COLOR_DRONE_PATH)
    ax.text(fixed_station_x[0] + 0.5, fixed_station_x[1] + 0.6, '无人机降落 (固定站点)', fontsize=9,
            color=COLOR_DRONE_PATH)
    ax.text(node2_launch[0] + 0.5, node2_launch[1] + 0.15, '卡车继续行驶', fontsize=9, color=COLOR_TRUCK)


if __name__ == '__main__':
    # 创建统一的画板 (10宽，14高) 和 3个垂直堆叠的子图
    fig, axes = plt.subplots(3, 1, figsize=(10, 14))
    fig.suptitle("卡车-无人机协同工作模式", fontsize=18, fontweight='bold', y=0.98)

    # 依次在对应的子图上进行绘制
    plot_synchronous_mode(axes[0])
    plot_asynchronous_mode(axes[1])
    plot_fixed_station_mode(axes[2])

    # 调整布局，防止重叠，并为总标题预留空间
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
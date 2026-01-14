import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from matplotlib.collections import LineCollection
import numpy as np


def simple_plot_route(result, coords_list, distmat=None, save_path='tsp_route_beautiful.png'):
    """
    美化版 TSP 可视化 (无 Colorbar 版，箭头增强版)
    """
    # 1. 重建路径逻辑
    succ = {int(i): int(j) for i, j in result}
    N = len(coords_list)

    if len(succ) != N:
        print("警告：后继数量 != 城市数，可能存在重复或缺失。仍尝试重建。")

    # 寻找起点 (默认 0，如果不在 keys 里则取第一个)
    start = 0 if 0 in succ else next(iter(succ))
    route = [start]

    # 遍历路径
    for _ in range(N):
        nxt = succ.get(route[-1])
        if nxt is None:
            print("路径中断，无法重建完整循环。")
            return
        route.append(nxt)

    # 检查闭环
    if route[0] != route[-1]:
        print("警告：未构成闭合回路，绘图可能不完整。")

    # 2. 计算总距离
    total_dist = 0.0
    if distmat is not None:
        total_dist = sum(distmat[a, b] for a, b in zip(route[:-1], route[1:]))
        print(f"总距离: {total_dist:.2f}")

    # --- 绘图设置 ---
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(10, 8))

    # 提取坐标
    xs = np.array([coords_list[i][0] for i in route])
    ys = np.array([coords_list[i][1] for i in route])

    # --- A. 绘制渐变色路径 ---
    points = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # 创建颜色映射
    cmap = plt.get_cmap('cool')
    norm = plt.Normalize(0, N)

    # 创建 LineCollection
    lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=2.5, alpha=0.8, zorder=1)
    lc.set_array(np.arange(N))  # 根据顺序上色
    ax.add_collection(lc)

    # --- B. 绘制箭头 (辅助方向 - 增强版) ---
    for i in range(len(route) - 1):
        # 只有当节点数非常多时才跳过部分箭头，否则尽量都画
        if N > 30 and i % 2 != 0: continue

        idx_start = route[i]
        idx_end = route[i + 1]
        x1, y1 = coords_list[idx_start]
        x2, y2 = coords_list[idx_end]

        # 计算箭头位置：放在线段的 55% 处
        mx = x1 + (x2 - x1) * 0.55
        my = y1 + (y2 - y1) * 0.55

        # 箭尾位置：放在线段的 45% 处 (这样箭头长度适中)
        tx = x1 + (x2 - x1) * 0.45
        ty = y1 + (y2 - y1) * 0.45

        ax.annotate('',
                    xy=(mx, my),
                    xytext=(tx, ty),
                    arrowprops=dict(
                        arrowstyle='-|>',  # 实心三角箭头，比 -> 更清晰
                        color='#222222',  # 深灰色/接近黑色
                        lw=2,  # 线条加粗
                        alpha=0.9,  # 不透明度提高
                        mutation_scale=20  # 箭头头部放大 (关键参数)
                    ),
                    zorder=2)

    # --- C. 绘制节点 ---
    all_indices = list(range(N))
    other_nodes = [i for i in all_indices if i != start]

    other_xs = [coords_list[i][0] for i in other_nodes]
    other_ys = [coords_list[i][1] for i in other_nodes]

    start_x, start_y = coords_list[start]

    # 绘制普通节点
    ax.scatter(other_xs, other_ys, c='white', edgecolors='#333333', s=80, linewidth=1.5, zorder=3, label='Customer')

    # 绘制起点/Depot
    ax.scatter(start_x, start_y, c='#FFD700', edgecolors='#FF4500', marker='*', s=250, linewidth=1.5, zorder=4,
               label='Depot')

    # --- D. 添加标签 ---
    for idx, (x, y) in enumerate(coords_list):
        ax.annotate(
            str(idx),
            xy=(x, y),
            xytext=(0, 0),
            textcoords='offset points',
            fontsize=9,
            fontweight='bold',
            color='#333333',
            zorder=5,
            ha='center',
            va='center'
        )

    # --- E. 图表装饰 ---
    title_text = f"TSP Route Visualization (N={N})"
    if distmat is not None:
        title_text += f"\nTotal Distance: {total_dist:.2f}"

    ax.set_title(title_text, fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel('X Coordinate', fontsize=10)
    ax.set_ylabel('Y Coordinate', fontsize=10)

    ax.set_aspect('equal', adjustable='datalim')

    # 移除顶部和右侧的边框
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"图片已保存至: {save_path}")
    plt.show()


# --- 测试代码 ---
if __name__ == "__main__":
    # 示例数据
    test_coords = [(0, 0), (2, 4), (5, 2), (6, 6), (8, 3), (9, 8), (4, 9), (1, 7), (3, 5), (7, 1)]
    test_result = [(0, 1), (1, 7), (7, 6), (6, 5), (5, 3), (3, 2), (2, 9), (9, 4), (4, 8), (8, 0)]

    import math

    N = len(test_coords)
    dist_matrix = {}
    for i in range(N):
        for j in range(N):
            d = math.sqrt((test_coords[i][0] - test_coords[j][0]) ** 2 + (test_coords[i][1] - test_coords[j][1]) ** 2)
            dist_matrix[i, j] = d

    simple_plot_route(test_result, test_coords, distmat=dist_matrix)
import matplotlib.pyplot as plt
import numpy as np

# 1. 整理提取数据
data = {
    'N': {
        'x': ['50', '75', '100', '125', '150'],
        'full': [1.08, 3.84, 9.66, 22.52,43.07],
        'eff': [0.65, 1.9, 4.26, 8.98, 16.1],
        'speedup': [1.66, 2.02, 2.27, 2.51, 2.68]
    },
    'm': {
        'x': ['1', '2', '3'],
        'full': [17.51, 15.70, 14.90],
        'eff': [6.25, 6.36, 6.52],
        'speedup': [2.8, 2.47, 2.28]
    },
    'L': {
        'x': ['1/2 AVG', 'AVG'],
        'full': [15.73, 16.34],
        'eff': [5.89, 6.86],
        'speedup': [2.67, 2.38]
    },
    'Tmax': {
        'x': ['1/3 TSP', '2/3 TSP'],
        'full': [6.9, 25.17],
        'eff': [3.31, 9.45],
        'speedup': [2.09, 2.66]
    }
}

# 2. 设置学术全局样式
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['font.size'] = 12
plt.rcParams['axes.linewidth'] = 1.2

# 3. 创建 2x2 的画布
fig, axes = plt.subplots(2, 2, figsize=(12, 10))
axes = axes.flatten()

# 定义图表标题和X轴标签
params = [
    ('N', '(a) Impact of Number of Nodes ($N$)', 'Number of Nodes ($N$)'),
    ('m', '(b) Impact of Number of Drones ($m$)', 'Number of Drones ($m$)'),
    ('L', '(c) Impact of Battery Endurance ($L$)', 'Battery Endurance ($L$)'),
    ('Tmax', '(d) Impact of Time Budget ($T_{max}$)', 'Time Budget ($T_{max}$)')
]

# 4. 封装绘图逻辑
width = 0.3  # 柱子宽度

for i, (key, title, xlabel) in enumerate(params):
    ax1 = axes[i]
    d = data[key]
    x = np.arange(len(d['x']))

    # === 左轴：绘制耗时（柱状图） ===
    bar1 = ax1.bar(x - width / 2, d['full'], width, label='VNS-fullEval', color='#4C72B0', edgecolor='black',
                   zorder=3)
    bar2 = ax1.bar(x + width / 2, d['eff'], width, label='VNS', color='#55A868', edgecolor='black',
                   zorder=3)

    ax1.set_xlabel(xlabel, fontsize=13, fontweight='bold')
    ax1.set_ylabel('Computation Time (s)', fontsize=13)
    ax1.set_title(title, fontsize=14, pad=10)
    ax1.set_xticks(x)
    ax1.set_xticklabels(d['x'])
    ax1.grid(axis='y', linestyle='--', alpha=0.5, zorder=0)

    max_time = max(d['full'])
    ax1.set_ylim(0, max_time * 1.25)  # 稍微调高一点顶部空间，防止遮挡数值

    # === 右轴：绘制加速比（折线图） ===
    ax2 = ax1.twinx()
    line1 = ax2.plot(x, d['speedup'], marker='o', markersize=8, linewidth=2.5,
                     label='Speedup', color='#C44E52', zorder=4)

    ax2.set_ylabel('Speedup', fontsize=13)

    min_speedup = min(d['speedup'])
    max_speedup = max(d['speedup'])
    # ax2.set_ylim(min_speedup - 0.5, max_speedup + 0.5)
    ax2.set_ylim(1.6, 3.0)

    # 标注具体的加速比数值
    for j, val in enumerate(d['speedup']):
        ax2.annotate(f"{val:.2f}x",
                     (x[j], val),
                     textcoords="offset points",
                     xytext=(0, 10),
                     ha='center', fontsize=11, fontweight='bold', color='#C44E52')

    # ====================== 关键修改：仅在第一个子图中添加 Legend ======================
    if i == 0:
        h1, l1 = ax1.get_legend_handles_labels()
        h2, l2 = ax2.get_legend_handles_labels()
        # 将左轴(柱状图)和右轴(折线图)的图例合并，放置在左上角 (upper left)
        # 字体稍微调到 11，防止图例框过大遮挡数据
        ax1.legend(h1 + h2, l1 + l2, loc='upper left', fontsize=11, framealpha=0.9, edgecolor='black')

# 5. 调整布局并保存
plt.tight_layout()
# 恢复默认的子图间距（因为顶部不再需要给全局图例留大段空白了）
fig.subplots_adjust(hspace=0.3, wspace=0.3)

plt.savefig('VNS_4_subplots_legend_inside.pdf', format='pdf', bbox_inches='tight')
plt.savefig('VNS_4_subplots_legend_inside.png', dpi=300, bbox_inches='tight')
plt.show()
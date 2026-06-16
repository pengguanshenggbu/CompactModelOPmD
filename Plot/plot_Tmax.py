import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']

# 准备数据结构
# 每个场景：[时间窗, 规模, OP得分, OPmD得分, 颜色]
# 为了美观，我们根据时间窗来给线段上色：0.33用红色，0.50用紫色，0.67用蓝色
scenarios = [
    [0.48, 0.61, '#e74c3c', '0.33 Tmax (N=50)'],
    [0.49, 0.62, '#e74c3c', '0.33 Tmax (N=100)'],
    [0.50, 0.62, '#e74c3c', '0.33 Tmax (N=150)'],

    [0.69, 0.83, '#9b59b6', '0.50 Tmax (N=50)'],
    [0.72, 0.85, '#9b59b6', '0.50 Tmax (N=100)'],
    [0.70, 0.83, '#9b59b6', '0.50 Tmax (N=150)'],

    [0.88, 0.97, '#3498db', '0.67 Tmax (N=50)'],
    [0.89, 0.97, '#3498db', '0.67 Tmax (N=100)'],
    [0.88, 0.96, '#3498db', '0.67 Tmax (N=150)'],
]

fig, ax = plt.subplots(figsize=(9, 6))

# 绘制纵向的虚拟轴线
ax.axvline(0, color='black', lw=1, alpha=0.7)
ax.axvline(1, color='black', lw=1, alpha=0.7)

# 循环绘制每个场景的演进折线
for op_val, opmd_val, color, label in scenarios:
    ax.plot([0, 1], [op_val, opmd_val], color=color, linewidth=2, alpha=0.8, marker='o', markersize=6)
    # 在右侧终点顺便标出具体的提升效果
    diff = opmd_val - op_val
    ax.text(1.03, opmd_val, f'{opmd_val:.2f} (+{diff * 100:.0f}%)', va='center', ha='left', color=color, fontsize=9,
            fontweight='bold')
    ax.text(-0.03, op_val, f'{op_val:.2f}', va='center', ha='right', color=color, fontsize=9)

# 轴美化
ax.set_xlim(-0.15, 1.25)
ax.set_ylim(0.4, 1.05)
ax.set_xticks([0, 1])
ax.set_xticklabels(['OP Model\n(Truck Only)', 'OP-mD Model\n(Truck + Drones)'], fontsize=12, fontweight='bold')
ax.set_ylabel('Profit Ratio', fontsize=12, fontweight='bold')
ax.grid(axis='y', linestyle='--', alpha=0.5)

# 制造手动的清晰图例
from matplotlib.lines import Line2D

custom_lines = [Line2D([0], [0], color='#e74c3c', lw=2),
                Line2D([0], [0], color='#9b59b6', lw=2),
                Line2D([0], [0], color='#3498db', lw=2)]
ax.legend(custom_lines, [r'0.33 $T_{\max}$ Scenario', r'0.50 $T_{\max}$ Scenario', r'0.67 $T_{\max}$ Scenario'],
          loc='upper left')

plt.title('Parallel Coordinates of Profit Progression from OP to OP-mD', fontsize=13, fontweight='bold', pad=15)
plt.tight_layout()
plt.savefig('parallel_coordinates_profit.png', dpi=300)
plt.show()
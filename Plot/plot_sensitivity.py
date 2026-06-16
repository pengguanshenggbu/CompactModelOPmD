import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import Normalize
import matplotlib.cm as cm
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

# 1. 结构化你的真实实验数据
data_str = """alpha	L	m	ratio
0.5	0.5	1	0.50
0.5	0.5	2	0.50
0.5	0.5	3	0.51
0.5	0.5	4	0.51
0.5	0.75	1	0.51
0.5	0.75	2	0.52
0.5	0.75	3	0.53
0.5	0.75	4	0.54
0.5	1	1	0.51
0.5	1	2	0.53
0.5	1	3	0.53
0.5	1	4	0.54
0.5	1.25	1	0.52
0.5	1.25	2	0.53
0.5	1.25	3	0.55
0.5	1.25	4	0.56
1	0.5	1	0.52
1	0.5	2	0.53
1	0.5	3	0.54
1	0.5	4	0.54
1	0.75	1	0.53
1	0.75	2	0.56
1	0.75	3	0.57
1	0.75	4	0.59
1	1	1	0.54
1	1	2	0.57
1	1	3	0.60
1	1	4	0.62
1	1.25	1	0.54
1	1.25	2	0.58
1	1.25	3	0.62
1	1.25	4	0.65
1.5	0.5	1	0.56
1.5	0.5	2	0.58
1.5	0.5	3	0.59
1.5	0.5	4	0.60
1.5	0.75	1	0.60
1.5	0.75	2	0.64
1.5	0.75	3	0.68
1.5	0.75	4	0.68
1.5	1	1	0.61
1.5	1	2	0.69
1.5	1	3	0.74
1.5	1	4	0.77
1.5	1.25	1	0.61
1.5	1.25	2	0.70
1.5	1.25	3	0.76
1.5	1.25	4	0.81
2	0.5	1	0.59
2	0.5	2	0.60
2	0.5	3	0.61
2	0.5	4	0.61
2	0.75	1	0.63
2	0.75	2	0.71
2	0.75	3	0.71
2	0.75	4	0.72
2	1	1	0.67
2	1	2	0.77
2	1	3	0.80
2	1	4	0.82
2	1.25	1	0.68
2	1.25	2	0.78
2	1.25	3	0.84
2	1.25	4	0.88"""

lines = [line.strip().split() for line in data_str.strip().split('\n')]
df = pd.DataFrame(lines[1:], columns=lines[0]).astype(float)

# 设置顶刊常用的 Times New Roman 学术字体
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['axes.unicode_minus'] = False

# 修改这里：使用 constrained_layout=True 替代 fig.tight_layout()
fig, ax = plt.subplots(figsize=(11, 9), subplot_kw={'projection': '3d'}, constrained_layout=True)

alpha = df['alpha'].values
L = df['L'].values
m = df['m'].values
ratio = df['ratio'].values

norm = Normalize(vmin=0.45, vmax=0.90)
cmap = cm.YlOrRd

# 绘制数据散点云
sc = ax.scatter3D(
    alpha, L, m, c=ratio, cmap=cmap, norm=norm,
    s=ratio * 450, alpha=0.85, edgecolors='black', linewidths=0.6, zorder=5
)

# 2. 坐标轴美化与留白 (Padding) 设定
ax.set_xticks([0.5, 1.0, 1.5, 2.0])
ax.set_xticklabels(['0.5x', '1.0x', '1.5x', '2.0x'])
ax.set_xlabel('Drone Speed Ratio ($\\alpha$)', fontsize=11, fontweight='bold', labelpad=12)

ax.set_yticks([0.5, 0.75, 1.0, 1.25])
ax.set_yticklabels(['0.5', '0.75', '1.0', '1.25'])
ax.set_ylabel('Drone Endurance (L/AVG)', fontsize=11, fontweight='bold', labelpad=12)

ax.set_zticks([1, 2, 3, 4])
ax.set_zticklabels(['m=1', 'm=2', 'm=3', 'm=4'])
ax.set_zlabel('Number of Drones (m)', fontsize=11, fontweight='bold', labelpad=8)

# 3. 构造完美的 3D 渲染类，防止箭头乱飞或越界
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)

# 4. 校准箭头的物理终点，在边界内测留出优雅的安全距离
# 路径1（蓝色）：从 (0.5, 0.5, 1) 横移至靠近边界的 1.9
arrow1 = Arrow3D([0.5, 1.9], [0.5, 0.5], [1, 1], mutation_scale=15, lw=3.5, arrowstyle="-|>", color='#2980b9', zorder=15)
# 路径2（紫色）：从 (2.0, 0.5, 1) 纵深移至 1.15
arrow2 = Arrow3D([2.0, 2.0], [0.5, 1.15], [1, 1], mutation_scale=15, lw=3.5, arrowstyle="-|>", color='#8e44ad', zorder=15)
# 路径3（橙色）：从 (2.0, 1.25, 1) 垂直向上移至 3.7
arrow3 = Arrow3D([2.0, 2.0], [1.25, 1.25], [1, 3.7], mutation_scale=15, lw=3.5, arrowstyle="-|>", color='#d35400', zorder=15)

ax.add_artist(arrow1)
ax.add_artist(arrow2)
ax.add_artist(arrow3)

# 5. 调整文本标签排版，防止其遮挡网格边缘
# ax.text(1.3, 0.55, 0.6, 'Ratio(0.50 -> 0.59)', color='#2980b9', fontweight='bold', fontsize=11, ha='center', va='top')
# ax.text(2, 0.85, 0.7, 'Ratio(0.59 -> 0.68)', color='#8e44ad', fontweight='bold', fontsize=11, ha='left', va='center')
# ax.text(2.1, 1.3, 1.5, 'Ratio(0.68 -> 0.88)', color='#d35400', fontweight='bold', fontsize=11, ha='left', va='center')

# 6. 添加颜色条
cbar = fig.colorbar(sc, ax=ax, shrink=0.5, pad=0.12)
cbar.set_label('Collected-Prize Fraction', fontsize=11, fontweight='bold')

# 7. 锁定黄金视角并修正比例畸变
ax.view_init(elev=22, azim=-62)
ax.set_box_aspect([1, 1, 0.8])  # 优化三维比例，压缩垂直轴高度使整体更紧凑

# 移除 fig.tight_layout()，因为它已经被 constrained_layout=True 替代
# fig.tight_layout()
plt.savefig('fig_sensitivity.png', dpi=600) # 生成无越界的高清大图
plt.show()
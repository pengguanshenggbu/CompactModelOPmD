import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import Normalize
import matplotlib.cm as cm

# 解决中文显示问题（指定使用黑体）
plt.rcParams['font.sans-serif'] = ['SimHei']
# 解决负号 '-' 显示为方块的问题
plt.rcParams['axes.unicode_minus'] = False
# 1. 结构化你的实验数据
# 建立轴标签映射：
# X轴：速度 alpha (1, 2) -> 对应索引 0, 1
# Y轴：续航 L (0.5, 1.0) -> 对应索引 0, 1
# Z轴：数量 m (1, 2, 3)   -> 对应索引 0, 1, 2

# 初始化一个 2x2x3 的三维网格用于存放 profit ratio
data_3d = np.zeros((2, 2, 3))

# 填入你的真实数据
data_3d[0, 0, 0] = 0.47  # alpha=1, L=0.5, m=1
data_3d[0, 0, 1] = 0.47  # alpha=1, L=0.5, m=2
data_3d[0, 0, 2] = 0.47  # alpha=1, L=0.5, m=3

data_3d[0, 1, 0] = 0.49  # alpha=1, L=1.0, m=1
data_3d[0, 1, 1] = 0.51  # alpha=1, L=1.0, m=2
data_3d[0, 1, 2] = 0.53  # alpha=1, L=1.0, m=3

data_3d[1, 0, 0] = 0.57  # alpha=2, L=0.5, m=1
data_3d[1, 0, 1] = 0.58  # alpha=2, L=0.5, m=2
data_3d[1, 0, 2] = 0.58  # alpha=2, L=0.5, m=3

data_3d[1, 1, 0] = 0.68  # alpha=2, L=1.0, m=1
data_3d[1, 1, 1] = 0.76  # alpha=2, L=1.0, m=2
data_3d[1, 1, 2] = 0.82  # alpha=2, L=1.0, m=3

# 2. 设置色彩映射 (Colormap)
# 使用学术界经典的 'YlOrRd' (黄-橙-红) 渐变，值越大颜色越深越红
norm = Normalize(vmin=0.45, vmax=0.85)
colors = cm.YlOrRd(norm(data_3d))
# 设置体素的透明度 alpha（注意此 alpha 是绘图透明度，非速度参数），使内部结构隐约可见
colors[..., 3] = 0.75

# 3. 开始绘制三维体素图
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# 所有的体素都激活绘制 (2x2x3 的网格全填满，靠颜色区分)
filled = np.ones((2, 2, 3), dtype=bool)

# 绘制三维体素
ax.voxels(filled, facecolors=colors, edgecolor='grey', linewidth=0.5)

# 4. 坐标轴美化与标签设定
# 由于 ax.voxels 绘制在单位网格上，我们需要将刻度定位在方块的中心或边缘
ax.set_xticks([0.5, 1.5])
ax.set_xticklabels(['1 (低速)', '2 (高速)'], fontsize=10)
ax.set_xlabel('无人机速度倍率 (alpha)', fontsize=11)

fig.show()
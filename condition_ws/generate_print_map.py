#!/usr/bin/env python3
"""
测试场地打印图 - 仅边界+障碍物
场地尺寸: 2.8m × 1.8m
打印时告知打印店: 按 280cm × 180cm 打印，不缩放
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

FIELD_W = 2.8
FIELD_H = 1.8

# 障碍物 (x_min, y_min, width, height)
obstacles = [
    (0.00, 0.00, 0.50, 1.30),   # 左块
    (1.30, 0.55, 0.40, 1.25),   # 中块
    (2.25, 0.00, 0.55, 1.05),   # 右下块
    (2.25, 1.62, 0.55, 0.18),   # 右上角
    (1.20, 0.85, 0.10, 0.25),   # 小凸块
]

C_ROAD   = '#FFFFFF'   # 道路：白色
C_OBS    = '#222222'   # 障碍物：深灰近黑
C_BORDER = '#000000'   # 外框线：黑色

# 图像尺寸：保持 2.8:1.8 比例，300 DPI
# figsize=(14, 9) → 4200×2700 像素
# figsize=(14, 9) 比例恰好等于 2.8:1.8，add_axes([0,0,1,1]) 让坐标系撑满整张图
# → 输出像素中不含任何白边，每像素严格对应相同的物理比例
fig = plt.figure(figsize=(14, 9), dpi=300)
ax = fig.add_axes([0, 0, 1, 1])   # 填满整个 figure，无边距

ax.set_xlim(0, FIELD_W)
ax.set_ylim(0, FIELD_H)
# 不用 set_aspect('equal')：figsize 比例已与数据范围一致(14/9 = 2.8/1.8)
ax.axis('off')

# 场地背景（道路/通道区域）
ax.add_patch(patches.Rectangle((0, 0), FIELD_W, FIELD_H,
             linewidth=0, facecolor=C_ROAD, zorder=1))

# 障碍物：只填色，不画边框线
# 这样障碍物与外框相交处、障碍物互相接触处都不会出现多余分割线
for x, y, w, h in obstacles:
    ax.add_patch(patches.Rectangle((x, y), w, h,
                 linewidth=0, facecolor=C_OBS, zorder=2))

# 不画外框线——图片四条边本身就是场地边界，打印时图片边缘即为 280×180cm 的边

# pad_inches=0 + 不用 bbox_inches='tight' → 严格按 figsize 输出，零白边
plt.savefig('/home/yssun/Workspace/condition_ws/field_print.png',
            dpi=300, pad_inches=0, facecolor=C_ROAD)
plt.savefig('/home/yssun/Workspace/condition_ws/field_print.svg',
            format='svg', pad_inches=0, facecolor=C_ROAD)

print("done: field_print.png / field_print.svg")
print("打印尺寸: 280cm × 180cm，不缩放")

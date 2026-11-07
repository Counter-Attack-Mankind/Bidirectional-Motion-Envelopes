#!/usr/bin/env python3
"""场景可视化脚本：验证障碍物布局和车辆通行性
基于会议截图重新设计：3道墙(交替豁口) + 2个角落块 = 5个AABB障碍物，3个弯"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json, os
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK JP']
matplotlib.rcParams['axes.unicode_minus'] = False

# ============ 车辆参数 ============
LENGTH = 0.211
WIDTH  = 0.191
WHEELBASE = 0.143
REAR_HANG = 0.032
FRONT_HANG = 0.036
MAX_PHI = 0.3491
MAX_V = 0.25

R_MIN = WHEELBASE / np.tan(MAX_PHI)
R_OUTER = np.sqrt((R_MIN + WIDTH/2)**2 + (WHEELBASE + FRONT_HANG)**2)

# ============ 场地参数 ============
FIELD_W = 2.80
FIELD_H = 1.80

# ============ 障碍物布局 (v2: 3墙+2角落块) ============
# 计算: 3道墙(0.10m厚) + 4条通道 = 1.80m
# 通道宽: (1.80 - 0.30) / 4 = 0.375m (车宽0.191m, 两侧余0.092m)
# 墙交替从左/右延伸, 1.0m宽豁口

WALL_THICK = 0.10
CORRIDOR_H = 0.375

# 墙的y坐标
wall1_bot = CORRIDOR_H                              # 0.375
wall2_bot = CORRIDOR_H + WALL_THICK + CORRIDOR_H    # 0.85
wall3_bot = wall2_bot + WALL_THICK + CORRIDOR_H     # 1.325

# 通道
corridors = [
    ("C1-底部通道", 0, wall1_bot, CORRIDOR_H),
    ("C2-中下通道", wall1_bot + WALL_THICK, wall2_bot, CORRIDOR_H),
    ("C3-中上通道", wall2_bot + WALL_THICK, wall3_bot, CORRIDOR_H),
    ("C4-顶部通道", wall3_bot + WALL_THICK, FIELD_H, CORRIDOR_H),
]

GAP_W = 1.00  # 豁口宽度(供转弯)
WALL_LEN = FIELD_W - GAP_W  # 墙长 = 1.80m

obstacles = [
    # obs1: 墙A - 从左侧延伸, 豁口在右 (弯1)
    [[0.00, wall1_bot], [WALL_LEN, wall1_bot],
     [WALL_LEN, wall1_bot + WALL_THICK], [0.00, wall1_bot + WALL_THICK]],

    # obs2: 墙B - 从右侧延伸, 豁口在左 (弯2)
    [[GAP_W, wall2_bot], [FIELD_W, wall2_bot],
     [FIELD_W, wall2_bot + WALL_THICK], [GAP_W, wall2_bot + WALL_THICK]],

    # obs3: 墙C - 从左侧延伸, 豁口在右 (弯3)
    [[0.00, wall3_bot], [WALL_LEN, wall3_bot],
     [WALL_LEN, wall3_bot + WALL_THICK], [0.00, wall3_bot + WALL_THICK]],

    # obs4: 右侧角落块 (弯1转弯处, 缩窄豁口增加约束)
    [[2.50, wall1_bot + WALL_THICK], [FIELD_W, wall1_bot + WALL_THICK],
     [FIELD_W, wall1_bot + WALL_THICK + 0.15], [2.50, wall1_bot + WALL_THICK + 0.15]],

    # obs5: 左侧角落块 (弯2转弯处)
    [[0.00, wall2_bot - 0.15], [0.30, wall2_bot - 0.15],
     [0.30, wall2_bot], [0.00, wall2_bot]],
]

obs_names = [
    "obs1: 墙A (豁口右)",
    "obs2: 墙B (豁口左)",
    "obs3: 墙C (豁口右)",
    "obs4: 右角落块",
    "obs5: 左角落块",
]

# 起点/终点
START = [0.20, CORRIDOR_H / 2, 0.0]  # 底部通道左端，朝右
# 3道墙→车最终heading LEFT进入顶部通道，终点在左端
GOAL  = [0.20, wall3_bot + WALL_THICK + CORRIDOR_H / 2, np.pi]  # 顶部通道左端，朝左

# ============ 绘图 ============
def draw_vehicle(ax, x, y, theta, color='blue', alpha=0.5, label=None):
    cx_offset = LENGTH/2 - REAR_HANG
    cx = x + cx_offset * np.cos(theta)
    cy = y + cx_offset * np.sin(theta)
    corners_local = np.array([
        [-LENGTH/2, -WIDTH/2], [LENGTH/2, -WIDTH/2],
        [LENGTH/2, WIDTH/2], [-LENGTH/2, WIDTH/2],
    ])
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    corners = (R @ corners_local.T).T + np.array([cx, cy])
    poly = plt.Polygon(corners, closed=True, facecolor=color, alpha=alpha,
                        edgecolor=color, linewidth=1.5, label=label)
    ax.add_patch(poly)
    ax.plot(x, y, 'o', color=color, markersize=4)
    arrow_len = LENGTH * 0.6
    ax.annotate('', xy=(x + arrow_len*np.cos(theta), y + arrow_len*np.sin(theta)),
                xytext=(x, y), arrowprops=dict(arrowstyle='->', color=color, lw=2))

fig, ax = plt.subplots(1, 1, figsize=(14, 10))
ax.set_xlim(-0.20, FIELD_W + 0.20)
ax.set_ylim(-0.20, FIELD_H + 0.20)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3, linewidth=0.5)
ax.set_xlabel('X (m)', fontsize=12)
ax.set_ylabel('Y (m)', fontsize=12)
ax.set_title(f'场景布局 v2 | 5个AABB障碍物, 3个弯 | 场地{FIELD_W}×{FIELD_H}m | '
             f'车{LENGTH}×{WIDTH}m | R_min={R_MIN:.3f}m', fontsize=12)

# 场地边界
border = plt.Rectangle((0, 0), FIELD_W, FIELD_H, fill=False,
                         edgecolor='black', linewidth=2.5)
ax.add_patch(border)

# 通道着色和标注
corridor_colors = ['#E3F2FD', '#E8F5E9', '#FFF3E0', '#F3E5F5']
for i, (name, y_bot, y_top, h) in enumerate(corridors):
    rect = plt.Rectangle((0, y_bot), FIELD_W, h, facecolor=corridor_colors[i], alpha=0.4)
    ax.add_patch(rect)
    ax.text(-0.17, (y_bot + y_top) / 2, f'{name}\nh={h:.3f}m',
            fontsize=7, color='gray', ha='right', va='center', rotation=0)

# 障碍物
for i, obs in enumerate(obstacles):
    pts = np.array(obs)
    x_min, y_min = pts.min(axis=0)
    x_max, y_max = pts.max(axis=0)
    w_obs = x_max - x_min
    h_obs = y_max - y_min
    rect = plt.Rectangle((x_min, y_min), w_obs, h_obs,
                           facecolor='#D32F2F', alpha=0.75, edgecolor='#B71C1C', linewidth=1.5)
    ax.add_patch(rect)
    # 标注
    label = f'obs{i+1}'
    if h_obs > 0.12:
        label += f'\n{w_obs:.2f}×{h_obs:.2f}'
    ax.text((x_min+x_max)/2, (y_min+y_max)/2, label,
            ha='center', va='center', fontsize=7, color='white', fontweight='bold')

# 豁口标注
gaps = [
    (WALL_LEN + (FIELD_W - WALL_LEN)/2, wall1_bot + WALL_THICK/2,
     f"豁口1(右)\n{GAP_W:.1f}m", "#4CAF50"),
    (GAP_W/2, wall2_bot + WALL_THICK/2,
     f"豁口2(左)\n{GAP_W:.1f}m", "#4CAF50"),
    (WALL_LEN + (FIELD_W - WALL_LEN)/2, wall3_bot + WALL_THICK/2,
     f"豁口3(右)\n{GAP_W:.1f}m", "#4CAF50"),
]
for gx, gy, gtxt, gc in gaps:
    ax.annotate(gtxt, xy=(gx, gy), fontsize=8, color=gc, ha='center', va='center',
                fontweight='bold', bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.85))

# 起点/终点
draw_vehicle(ax, *START, color='#1565C0', alpha=0.6, label='START')
ax.annotate('START', xy=(START[0]+0.15, START[1]+0.12), fontsize=10,
            color='#1565C0', fontweight='bold')

draw_vehicle(ax, *GOAL, color='#2E7D32', alpha=0.6, label='GOAL')
ax.annotate('GOAL', xy=(GOAL[0]-0.15, GOAL[1]+0.12), fontsize=10,
            color='#2E7D32', fontweight='bold')

# 示意路径 (蛇形: →右 ↗弯1 ←左 ↗弯2 →右 ↗弯3 ←左)
c1_y = CORRIDOR_H / 2                                 # 底部通道中心
c2_y = wall1_bot + WALL_THICK + CORRIDOR_H / 2        # 中下通道中心
c3_y = wall2_bot + WALL_THICK + CORRIDOR_H / 2        # 中上通道中心
c4_y = wall3_bot + WALL_THICK + CORRIDOR_H / 2        # 顶部通道中心

path_points = np.array([
    [0.20, c1_y],        # start
    [0.80, c1_y],
    [1.40, c1_y],        # 底部通道向右
    [1.75, c1_y],
    [2.00, c1_y + 0.05], # 接近弯1
    [2.25, wall1_bot + WALL_THICK + 0.05],  # 进入右豁口
    [2.35, c2_y - 0.05], # 弯1转弯中
    [2.20, c2_y],        # 进入中下通道
    [1.80, c2_y],
    [1.20, c2_y],        # 中下通道向左
    [0.70, c2_y],
    [0.45, c2_y + 0.05], # 接近弯2
    [0.25, wall2_bot - 0.05], # 进入左豁口
    [0.20, c3_y - 0.05], # 弯2转弯中
    [0.30, c3_y],        # 进入中上通道
    [0.80, c3_y],
    [1.40, c3_y],        # 中上通道向右
    [1.75, c3_y],
    [2.00, c3_y + 0.05], # 接近弯3
    [2.25, wall3_bot + WALL_THICK + 0.05], # 进入右豁口
    [2.35, c4_y - 0.05], # 弯3转弯中
    [2.20, c4_y],        # 进入顶部通道
    [1.60, c4_y],
    [0.80, c4_y],        # 顶部通道向左
    [0.20, c4_y],        # goal
])
ax.plot(path_points[:, 0], path_points[:, 1], 'o-', color='#FF9800',
        markersize=2.5, linewidth=2.0, alpha=0.8, label='示意路径(3弯)')

# 弯的编号
bend_labels = [
    (2.30, (c1_y + c2_y)/2, "弯1"),
    (0.25, (c2_y + c3_y)/2, "弯2"),
    (2.30, (c3_y + c4_y)/2, "弯3"),
]
for bx, by, bt in bend_labels:
    ax.annotate(bt, xy=(bx, by), fontsize=11, color='#E65100', fontweight='bold',
                ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFF3E0', alpha=0.9))

# 方向箭头标注每条通道行驶方向
dir_arrows = [
    (0.90, c1_y, '→  C1: 向右'),
    (1.90, c2_y, '←  C2: 向左'),
    (0.90, c3_y, '→  C3: 向右'),
    (1.90, c4_y, '←  C4: 向左'),
]
for dx, dy, dt in dir_arrows:
    ax.text(dx, dy + 0.13, dt, fontsize=8, color='#37474F', ha='center',
            fontweight='bold', alpha=0.7)

# 场地尺寸标注
ax.annotate('', xy=(FIELD_W, -0.14), xytext=(0, -0.14),
            arrowprops=dict(arrowstyle='<->', color='gray', lw=1))
ax.text(FIELD_W/2, -0.12, f'{FIELD_W}m', fontsize=9, color='gray', ha='center')
ax.annotate('', xy=(-0.14, FIELD_H), xytext=(-0.14, 0),
            arrowprops=dict(arrowstyle='<->', color='gray', lw=1))
ax.text(-0.14, FIELD_H/2, f'{FIELD_H}m', fontsize=9, color='gray', ha='right',
        va='center', rotation=90)

# 参数信息框
info_text = (
    f"车辆: {LENGTH}×{WIDTH}m, 轴距{WHEELBASE}m\n"
    f"max_phi={np.degrees(MAX_PHI):.1f}°, max_v={MAX_V}m/s\n"
    f"R_min={R_MIN:.3f}m, R_outer={R_OUTER:.3f}m\n"
    f"通道宽={CORRIDOR_H:.3f}m (余量{(CORRIDOR_H-WIDTH)/2:.3f}m/侧)\n"
    f"豁口宽={GAP_W:.1f}m, 墙厚={WALL_THICK:.2f}m\n"
    f"障碍物: {len(obstacles)}个, 弯: 3个\n"
    f"S曲线水平距离≈{2*R_MIN*np.sin(np.arccos(1-(CORRIDOR_H+WALL_THICK)/(2*R_MIN))):.3f}m < 豁口{GAP_W}m ✓"
)
ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=7.5,
        verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.85))

ax.legend(loc='lower right', fontsize=9)

out_path = os.path.join(os.path.dirname(__file__), 'scene_layout.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"已保存: {out_path}")
print(f"\n=== 布局参数 ===")
print(f"通道高度: {CORRIDOR_H:.3f}m × 4条")
print(f"墙厚度: {WALL_THICK:.2f}m × 3道")
print(f"验证: {4*CORRIDOR_H + 3*WALL_THICK:.3f}m == {FIELD_H}m")
print(f"通道余量: {(CORRIDOR_H-WIDTH)/2:.4f}m/侧 ({(CORRIDOR_H-WIDTH)/WIDTH*100:.1f}%)")
print(f"S曲线转弯需要水平距离: {2*R_MIN*np.sin(np.arccos(1-(CORRIDOR_H+WALL_THICK)/(2*R_MIN))):.3f}m")
print(f"豁口宽度: {GAP_W:.1f}m → 裕度充足")

# 碰撞检查：确认起点/终点不与障碍物重叠
for name, pose in [("START", START), ("GOAL", GOAL)]:
    cx_off = LENGTH/2 - REAR_HANG
    cx = pose[0] + cx_off * np.cos(pose[2])
    cy = pose[1] + cx_off * np.sin(pose[2])
    for i, obs in enumerate(obstacles):
        pts = np.array(obs)
        ox_min, oy_min = pts.min(axis=0)
        ox_max, oy_max = pts.max(axis=0)
        # 简单AABB碰撞(用车辆AABB近似)
        vx_min = cx - LENGTH/2
        vx_max = cx + LENGTH/2
        vy_min = cy - WIDTH/2
        vy_max = cy + WIDTH/2
        if vx_max > ox_min and vx_min < ox_max and vy_max > oy_min and vy_min < oy_max:
            print(f"⚠️ {name} 与 obs{i+1} 碰撞!")

plt.close()

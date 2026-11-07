#!/usr/bin/env python3
"""可视化Hybrid A*搜索节点 + 场景，诊断搜索失败原因"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import csv, glob, os, json
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK JP']
matplotlib.rcParams['axes.unicode_minus'] = False

# ============ 车辆参数 ============
LENGTH = 0.211
WIDTH  = 0.191
WHEELBASE = 0.143
REAR_HANG = 0.032
MAX_PHI = 0.3491
R_MIN = WHEELBASE / np.tan(MAX_PHI)

# ============ 加载场景 ============
script_dir = os.path.dirname(os.path.abspath(__file__))
env_path = os.path.join(script_dir, '..', 'config', 'env.json')
with open(env_path) as f:
    env = json.load(f)

# ============ 找最新的搜索日志 ============
log_dir = '/tmp/condition_planner/log/hybrid_astar'
search_files = sorted(glob.glob(os.path.join(log_dir, '*_search_nodes.csv')))
coarse_files = sorted(glob.glob(os.path.join(log_dir, '*_coarse_path.csv')))

if not search_files:
    print(f"未找到搜索日志: {log_dir}/*_search_nodes.csv")
    exit(1)

latest_search = search_files[-1]
latest_coarse = coarse_files[-1] if coarse_files else None
print(f"搜索节点: {latest_search}")

# 读搜索节点
nodes_x, nodes_y, nodes_phi, nodes_dir = [], [], [], []
with open(latest_search) as f:
    reader = csv.DictReader(f)
    for row in reader:
        nodes_x.append(float(row['x']))
        nodes_y.append(float(row['y']))
        nodes_phi.append(float(row['phi']))
        nodes_dir.append(int(row['direction']))

print(f"共 {len(nodes_x)} 个搜索节点")

# 读粗解路径
coarse_x, coarse_y = [], []
if latest_coarse and os.path.getsize(latest_coarse) > 10:
    with open(latest_coarse) as f:
        reader = csv.DictReader(f)
        for row in reader:
            coarse_x.append(float(row['x']))
            coarse_y.append(float(row['y']))
    print(f"粗解路径: {len(coarse_x)} 点")

# ============ 统计 ============
nodes_x = np.array(nodes_x)
nodes_y = np.array(nodes_y)
nodes_phi = np.array(nodes_phi)

print(f"\n=== 搜索范围 ===")
print(f"  x: [{nodes_x.min():.3f}, {nodes_x.max():.3f}]")
print(f"  y: [{nodes_y.min():.3f}, {nodes_y.max():.3f}]")
print(f"  phi: [{nodes_phi.min():.3f}, {nodes_phi.max():.3f}] rad")
print(f"       [{np.degrees(nodes_phi.min()):.1f}, {np.degrees(nodes_phi.max()):.1f}] deg")

# 按y分布统计
y_bins = np.arange(0, 1.85, 0.1)
y_hist, _ = np.histogram(nodes_y, bins=y_bins)
print(f"\n=== y分布 (每0.1m) ===")
for i in range(len(y_hist)):
    bar = '█' * (y_hist[i] // 500)
    print(f"  y=[{y_bins[i]:.1f},{y_bins[i+1]:.1f}): {y_hist[i]:6d}  {bar}")

# 检查是否有节点进入豁口区域
wall_a_top = 0.50
wall_b_bot = 1.30
gap_right_mask = (nodes_x > 1.60) & (nodes_y > wall_a_top)
gap_left_mask = (nodes_x < 1.20) & (nodes_y > wall_a_top)
middle_mask = (nodes_y > wall_a_top) & (nodes_y < wall_b_bot)

print(f"\n=== 区域分析 ===")
print(f"  底部通道 (y<{wall_a_top}): {np.sum(nodes_y < wall_a_top)} 节点")
print(f"  右豁口区 (x>1.60, y>{wall_a_top}): {np.sum(gap_right_mask)} 节点")
print(f"  中间通道 (y>{wall_a_top} & y<{wall_b_bot}): {np.sum(middle_mask)} 节点")
print(f"  左豁口区 (x<1.20, y>{wall_a_top}): {np.sum(gap_left_mask)} 节点")
print(f"  上部区域 (y>{wall_b_bot}): {np.sum(nodes_y > wall_b_bot)} 节点")

# ============ 绘图 ============
fig, axes = plt.subplots(1, 2, figsize=(20, 8))

for ax_idx, ax in enumerate(axes):
    ax.set_xlim(-0.1, 2.9)
    ax.set_ylim(-0.1, 1.9)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2, linewidth=0.3)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    # 场地边界
    ax.add_patch(plt.Rectangle((0, 0), 2.80, 1.80, fill=False, edgecolor='black', lw=2))

    # 障碍物
    for obs in env['obstacles']:
        pts = np.array(obs)
        poly = plt.Polygon(pts, closed=True, facecolor='#D32F2F', alpha=0.6, edgecolor='#B71C1C', lw=1)
        ax.add_patch(poly)

    # 起终点
    start = env['start']
    goal = env['goal']
    ax.plot(start[0], start[1], 's', color='blue', markersize=8, label='Start')
    ax.plot(goal[0], goal[1], '*', color='green', markersize=12, label='Goal')

# 左图: 搜索节点按探索顺序着色
ax = axes[0]
ax.set_title(f'搜索节点 (按探索顺序, 共{len(nodes_x)}个)')
sc = ax.scatter(nodes_x, nodes_y, c=np.arange(len(nodes_x)), cmap='viridis',
                s=0.3, alpha=0.5)
plt.colorbar(sc, ax=ax, label='探索顺序', shrink=0.8)
if coarse_x:
    ax.plot(coarse_x, coarse_y, 'r-', lw=2, label='粗解路径')
ax.legend(loc='upper left', fontsize=8)

# 右图: 搜索节点按heading着色
ax = axes[1]
ax.set_title(f'搜索节点 (按heading着色)')
sc = ax.scatter(nodes_x, nodes_y, c=np.degrees(nodes_phi), cmap='hsv',
                s=0.3, alpha=0.5, vmin=-180, vmax=180)
plt.colorbar(sc, ax=ax, label='heading (deg)', shrink=0.8)
ax.legend(loc='upper left', fontsize=8)

plt.tight_layout()
out_path = os.path.join(script_dir, '..', 'config', 'hastar_search_debug.png')
plt.savefig(out_path, dpi=150, bbox_inches='tight')
print(f"\n已保存: {out_path}")
plt.close()

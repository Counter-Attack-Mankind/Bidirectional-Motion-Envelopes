#!/usr/bin/env python3
"""画PP跟踪效果: 规划轨迹 vs 实际轨迹"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import csv, json, os

matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK JP']
matplotlib.rcParams['axes.unicode_minus'] = False

script_dir = os.path.dirname(os.path.abspath(__file__))

# ============ 加载规划轨迹 ============
plan_csv = os.path.join(script_dir, '..', 'config', 'trajectory_v2.csv')
plan_x, plan_y, plan_v, plan_t = [], [], [], []
with open(plan_csv) as f:
    reader = csv.DictReader(f)
    for row in reader:
        plan_t.append(float(row['t']))
        plan_x.append(float(row['x']))
        plan_y.append(float(row['y']))
        plan_v.append(float(row['v']))

# ============ 加载PP跟踪日志 ============
log_csv = '/tmp/pp_tracking_log.csv'
act_t, act_x, act_y, act_yaw = [], [], [], []
ref_x, ref_y, throttle_list, steering_list = [], [], [], []
with open(log_csv) as f:
    for line in f:
        if line.startswith('time') or 'traj_sz=0' in line:
            continue
        parts = line.strip().split(',')
        if len(parts) < 14:
            continue
        t = float(parts[0])
        ox, oy, oyaw = float(parts[1]), float(parts[2]), float(parts[3])
        rx, ry = float(parts[4]), float(parts[5])
        thr = float(parts[11])
        steer = float(parts[12])
        if ox == 0 and oy == 0:
            continue
        if thr == 0 and rx == 0:  # debug行(非控制行)
            continue
        act_t.append(t)
        act_x.append(ox)
        act_y.append(oy)
        act_yaw.append(oyaw)
        ref_x.append(rx)
        ref_y.append(ry)
        throttle_list.append(thr)
        steering_list.append(steer)

# ============ 加载场景 ============
env_path = os.path.join(script_dir, '..', 'config', 'env.json')
with open(env_path) as f:
    env = json.load(f)

# ============ 画图 ============
fig, axes = plt.subplots(2, 2, figsize=(16, 12))

# --- 左上: XY轨迹对比 ---
ax = axes[0, 0]
ax.set_aspect('equal')
ax.set_title('Tracking: Plan vs Actual')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# 场地边界
ax.add_patch(plt.Rectangle((0, 0), 2.80, 1.80, fill=False, edgecolor='black', lw=1.5))
# 障碍物
for obs in env['obstacles']:
    pts = np.array(obs + [obs[0]])
    ax.fill(pts[:, 0], pts[:, 1], color='#D32F2F', alpha=0.4)
    ax.plot(pts[:, 0], pts[:, 1], color='#B71C1C', lw=1)

ax.plot(plan_x, plan_y, 'b-', lw=1.5, alpha=0.7, label='Plan')
ax.plot(act_x, act_y, 'r-', lw=1.5, alpha=0.8, label='Actual')
ax.plot(plan_x[0], plan_y[0], 'bs', markersize=8, label='Start')
ax.plot(plan_x[-1], plan_y[-1], 'b*', markersize=10, label='Goal')
ax.plot(act_x[-1], act_y[-1], 'r*', markersize=10, label='Final pos')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# --- 右上: 横向误差 ---
ax = axes[0, 1]
ax.set_title('Lateral Error')
ax.set_xlabel('Elapsed (s)')
ax.set_ylabel('Error (m)')

# 计算与最近规划点的距离
lateral_err = []
for x, y in zip(act_x, act_y):
    dists = [(x - px)**2 + (y - py)**2 for px, py in zip(plan_x, plan_y)]
    lateral_err.append(np.sqrt(min(dists)))
ax.plot(act_t, lateral_err, 'r-', lw=1)
ax.axhline(y=0.05, color='orange', linestyle='--', label='5cm')
ax.axhline(y=0.10, color='red', linestyle='--', label='10cm')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
if lateral_err:
    ax.set_ylim(0, min(max(lateral_err) * 1.2, 0.5))

# --- 左下: 速度/油门 ---
ax = axes[1, 0]
ax.set_title('Velocity & Throttle')
ax.set_xlabel('Elapsed (s)')
ax.plot(act_t, throttle_list, 'g-', lw=1, label='Throttle (cmd)')
ax.plot(plan_t, plan_v, 'b--', lw=1, alpha=0.6, label='Plan v')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# --- 右下: 转向 ---
ax = axes[1, 1]
ax.set_title('Steering')
ax.set_xlabel('Elapsed (s)')
ax.set_ylabel('Steering (rad)')
ax.plot(act_t, steering_list, 'm-', lw=1)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = os.path.join(script_dir, '..', 'config', 'tracking_result.png')
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f"Saved: {out}")
print(f"\nStats:")
if lateral_err:
    print(f"  Lateral error: mean={np.mean(lateral_err):.4f}m, max={np.max(lateral_err):.4f}m")
print(f"  Duration: plan={plan_t[-1]:.1f}s, actual={act_t[-1]:.1f}s")
print(f"  Final pos: ({act_x[-1]:.3f}, {act_y[-1]:.3f}), goal=({plan_x[-1]:.3f}, {plan_y[-1]:.3f})")
print(f"  Final dist to goal: {np.sqrt((act_x[-1]-plan_x[-1])**2+(act_y[-1]-plan_y[-1])**2):.4f}m")
plt.close()

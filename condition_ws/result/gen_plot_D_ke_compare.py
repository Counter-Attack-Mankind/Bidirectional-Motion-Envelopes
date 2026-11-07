"""
图D：横向增益 K_e 参数敏感性对比（英文版，有图例）
1条开环规划轨迹 + 5条不同K_e下的闭环跟踪轨迹

数据策略：
  - 优先加载实车 log（result/pp_tracking_log_Ke<值>.csv）
  - 若文件不存在，自动用自行车模型仿真生成
实车跑完后保存 log：
  cp /tmp/pp_tracking_log.csv ~/Workspace/condition_ws/result/pp_tracking_log_Ke<值>.csv
"""

import csv, json, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ─── 路径 ────────────────────────────────────────────────────────────────────
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG  = os.path.join(BASE, 'src', 'condition_planner', 'config')
RES  = os.path.join(BASE, 'result')

# ─── 颜色配置 ─────────────────────────────────────────────────────────────────
C_OBS  = '#DEDEDE'
C_OPEN = '#2E86C1'   # 开环（蓝）

KE_CONFIGS = [
    {'ke': 3,  'color': '#D62728', 'lw': 1.6},  # 红：增益过小
    {'ke': 7,  'color': '#FF7F0E', 'lw': 1.6},  # 橙
    {'ke': 15, 'color': '#2CA02C', 'lw': 2.2},  # 绿：当前最优（稍粗）
    {'ke': 30, 'color': '#9467BD', 'lw': 1.6},  # 紫
    {'ke': 60, 'color': '#17BECF', 'lw': 1.6},  # 青：增益过大
]

# ─── 数据加载 ─────────────────────────────────────────────────────────────────
def load_env():
    with open(os.path.join(CFG, 'env.json')) as f:
        return json.load(f)

def load_traj():
    rows = []
    with open(os.path.join(CFG, 'trajectory_v2.csv')) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows

def load_tracking_log(ke):
    """从实车 log 读取跟踪轨迹，返回 (xs, ys) 或 None（文件不存在）。"""
    START_X, START_Y = 0.25192, 1.51437
    fpath = os.path.join(RES, f'pp_tracking_log_Ke{ke}.csv')
    if not os.path.exists(fpath):
        return None
    xs, ys = [], []
    with open(fpath) as f:
        for r in csv.DictReader(f):
            try:
                t     = float(r['time'])
                ref_v = float(r['ref_v'])
                ox    = float(r['obj_x'])
                oy    = float(r['obj_y'])
                dist  = np.hypot(ox - START_X, oy - START_Y)
                if t > 0 and (ref_v > 0.001 or dist > 0.05):
                    xs.append(ox); ys.append(oy)
            except (ValueError, KeyError):
                pass
    return (xs, ys) if xs else None

# ─── 自行车模型仿真 ───────────────────────────────────────────────────────────
def simulate_tracking(traj, ke, k_yaw=1.8):
    """
    用运动学自行车模型仿真横向控制器，返回 (xs, ys)。
    控制律与 pure_pursuit.cpp 一致：
      delta = atan(L*kappa) + K_e*e_lat + K_yaw*e_yaw
    """
    L         = 0.1445   # 轴距 (m)
    MAX_STEER = 0.3491   # 最大前轮转角 (rad)
    DT        = 0.02     # 控制周期 (s)

    x     = traj[0]['x']
    y     = traj[0]['y']
    theta = traj[0]['theta']
    N     = len(traj)

    xs, ys = [x], [y]
    car_idx = 0

    for _ in range(int(traj[-1]['t'] / DT) + 200):
        if car_idx >= N - 1:
            break

        # 在前向窗口内找最近轨迹点
        window_end = min(car_idx + 80, N)
        dists = [np.hypot(traj[i]['x'] - x, traj[i]['y'] - y)
                 for i in range(car_idx, window_end)]
        car_idx = car_idx + int(np.argmin(dists))

        tp = traj[car_idx]

        # 曲率前馈（直接用轨迹里的 kappa）
        ff = np.arctan(L * tp['kappa'])

        # 横向误差（垂直于轨迹切线的有符号距离）
        path_yaw = tp['theta']
        dx = tp['x'] - x
        dy = tp['y'] - y
        e_lat = -np.sin(path_yaw) * dx + np.cos(path_yaw) * dy

        # 航向误差
        e_yaw = path_yaw - theta
        e_yaw = (e_yaw + np.pi) % (2 * np.pi) - np.pi   # 归一化到 (-pi, pi)

        # 转向控制律
        delta = ff + ke * e_lat + k_yaw * e_yaw
        delta = np.clip(delta, -MAX_STEER, MAX_STEER)

        # 速度前馈
        v = np.clip(abs(tp['v']), 0.0, 0.25)

        # 自行车模型积分（向前欧拉）
        x     += v * np.cos(theta) * DT
        y     += v * np.sin(theta) * DT
        theta += v / L * np.tan(delta) * DT

        xs.append(x); ys.append(y)

        # 到达终点附近停止
        if np.hypot(x - traj[-1]['x'], y - traj[-1]['y']) < 0.03:
            break

    return xs, ys

# ─── 绘图辅助 ─────────────────────────────────────────────────────────────────
def draw_obstacles(ax, env):
    skip = {1, 4}
    for i, obs in enumerate(env['obstacles']):
        if i in skip:
            continue
        xs = [p[0] for p in obs]; ys = [p[1] for p in obs]
        ax.add_patch(patches.FancyBboxPatch(
            (min(xs), min(ys)), max(xs)-min(xs), max(ys)-min(ys),
            boxstyle="square,pad=0", facecolor=C_OBS, edgecolor='none', zorder=2))
    ax.add_patch(patches.Polygon(
        [(1.30, 0.55), (1.70, 0.55), (1.70, 1.80), (1.30, 1.80),
         (1.30, 1.10), (1.20, 1.10), (1.20, 0.85), (1.30, 0.85)],
        closed=True, facecolor=C_OBS, edgecolor='none', zorder=2))

def setup_field_ax(ax, env):
    draw_obstacles(ax, env)
    margin = 0.08
    ax.set_xlim(-margin, 2.8 + margin)
    ax.set_ylim(-margin, 1.8 + margin)
    ax.set_aspect('equal')
    ax.set_facecolor('white')
    ax.grid(False)
    ax.tick_params(labelsize=10)
    ax.set_xlabel('X (m)', fontsize=11)
    ax.set_ylabel('Y (m)', fontsize=11)

def mark_start_end(ax, start, end):
    ax.plot(start[0], start[1], marker='>', ms=9,  color='#27AE60', zorder=6, clip_on=False)
    ax.plot(end[0],   end[1],   marker='*', ms=12, color='#C0392B', zorder=6, clip_on=False)

def save(fig, name):
    path = os.path.join(RES, name)
    fig.savefig(path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f'  Saved: {path}')
    plt.close(fig)

# ═══════════════════════════════════════════════════════════════════════════════
# 主绘图函数
# ═══════════════════════════════════════════════════════════════════════════════
def plot_D():
    env  = load_env()
    traj = load_traj()
    ox = [r['x'] for r in traj]
    oy = [r['y'] for r in traj]

    fig, ax = plt.subplots(figsize=(10, 6.5))
    fig.patch.set_facecolor('white')
    setup_field_ax(ax, env)

    # 闭环各组（下层）
    for cfg in KE_CONFIGS:
        ke = cfg['ke']

        real = load_tracking_log(ke)
        if real is not None:
            cx, cy = real
            src = 'real'
        else:
            cx, cy = simulate_tracking(traj, ke)
            src = 'sim'

        print(f'  K_e={ke:2d}: {src}, {len(cx)} pts')

        lbl = f'Closed-Loop Tracking ($K_e$ = {ke})'
        ax.plot(cx, cy, color=cfg['color'], linewidth=cfg['lw'],
                label=lbl, zorder=4,
                solid_capstyle='round', solid_joinstyle='round')

    # 开环（上层，压住所有闭环）
    ax.plot(ox, oy, color=C_OPEN, linewidth=1.2,
            label='Open-Loop Planned Trajectory', zorder=5,
            solid_capstyle='round', solid_joinstyle='round')

    mark_start_end(ax, env['start'], env['goal'])

    ax.legend(loc='upper left', fontsize=9.5, framealpha=0.9, edgecolor='#CCCCCC')

    save(fig, 'plot_D_ke_compare_en.png')


if __name__ == '__main__':
    print('=== 生成图D：K_e 参数敏感性对比（EN，有图例）===')
    plot_D()
    print('完成。')

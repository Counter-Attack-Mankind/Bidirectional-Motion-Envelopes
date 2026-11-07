"""
图D：航向增益 K_yaw 参数敏感性对比（英文版，有图例）
1条开环规划轨迹 + 5条不同K_yaw下的闭环跟踪轨迹

数据文件（result/tracking/ 目录下）：
  005_pp_tracking_log.csv  → K_yaw=0.05  ζ=0.017
  045_pp_tracking_log.csv  → K_yaw=0.45  ζ=0.153
  pp_tracking_log_v2.csv   → K_yaw=1.80  ζ=0.611  (最优)
  360_pp_tracking_log.csv  → K_yaw=3.60  ζ=1.223
  720_pp_tracking_log.csv  → K_yaw=7.20  ζ=2.445
"""

import csv, json, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ─── 路径 ────────────────────────────────────────────────────────────────────
BASE    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG     = os.path.join(BASE, 'src', 'condition_planner', 'config')
RES     = os.path.join(BASE, 'result')
TRACK   = os.path.join(RES, 'tracking')

# ─── 颜色配置 ─────────────────────────────────────────────────────────────────
C_OBS  = '#D5D8DC'   # 障碍物（浅灰）
C_OPEN = '#1A5276'   # 开环（深蓝）

# 发散色系：红(欠阻尼) → 橙 → 绿(最优) → 紫 → 灰(过阻尼)
# 绘制顺序：先画偏差最大的（底层），最优最后画（靠近开环层）
KYAW_CONFIGS = [
    # 绘制顺序决定叠放关系
    {'kyaw': 7.20, 'file': '720_pp_tracking_log.csv',
     'color': '#717D7E', 'lw': 1.6, 'alpha': 0.85, 'zorder': 3,
     'label': r'Closed-Loop Tracking ($K_\psi = 7.20$)'},
    {'kyaw': 3.60, 'file': '360_pp_tracking_log.csv',
     'color': '#7D3C98', 'lw': 1.6, 'alpha': 0.85, 'zorder': 3,
     'label': r'Closed-Loop Tracking ($K_\psi = 3.60$)'},
    {'kyaw': 0.05, 'file': '045_pp_tracking_log.csv',
     'color': '#C0392B', 'lw': 2.0, 'alpha': 1.00, 'zorder': 4,
     'label': r'Closed-Loop Tracking ($K_\psi = 0.45$)'},
    {'kyaw': 0.45, 'file': '090_pp_tracking_log.csv',
     'color': '#E67E22', 'lw': 1.6, 'alpha': 0.90, 'zorder': 4,
     'label': r'Closed-Loop Tracking ($K_\psi = 0.90$)'},
    {'kyaw': 1.80, 'file': 'pp_tracking_log_v2.csv',
     'color': '#1E8449', 'lw': 2.8, 'alpha': 1.00, 'zorder': 5,
     'label': r'Closed-Loop Tracking ($K_\psi = 1.80$)'},
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

def load_tracking(fpath):
    START_X, START_Y = 0.25192, 1.51437
    if not os.path.exists(fpath):
        print(f'  [SKIP] {os.path.basename(fpath)} not found')
        return [], []
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
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    # 保留四条边框，但淡化
    for spine in ax.spines.values():
        spine.set_linewidth(0.8)
        spine.set_edgecolor('#AAAAAA')

def mark_start_end(ax, start, end):
    ax.plot(start[0], start[1], marker='>', ms=9,  color='#27AE60', zorder=6, clip_on=False)
    ax.plot(end[0],   end[1],   marker='*', ms=12, color='#C0392B', zorder=6, clip_on=False)

def save(fig, name):
    path = os.path.join(RES, name)
    fig.savefig(path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f'  Saved: {path}')
    plt.close(fig)

# ═══════════════════════════════════════════════════════════════════════════════
# 主绘图
# ═══════════════════════════════════════════════════════════════════════════════
def plot_D():
    env  = load_env()
    traj = load_traj()
    ox = [r['x'] for r in traj]
    oy = [r['y'] for r in traj]

    fig, ax = plt.subplots(figsize=(10, 6.5))
    fig.patch.set_facecolor('white')
    setup_field_ax(ax, env)

    # 闭环各组（按 KYAW_CONFIGS 中定义的 zorder 叠放）
    for cfg in KYAW_CONFIGS:
        fpath = os.path.join(TRACK, cfg['file'])
        cx, cy = load_tracking(fpath)
        if not cx:
            continue
        ax.plot(cx, cy,
                color=cfg['color'],
                linewidth=cfg['lw'],
                alpha=cfg['alpha'],
                label=cfg['label'],
                zorder=cfg['zorder'],
                solid_capstyle='round',
                solid_joinstyle='round')
        print(f'  K_yaw={cfg["kyaw"]:.2f}: {len(cx)} pts')

    # 开环（最上层）—— 深蓝细线
    ax.plot(ox, oy, color=C_OPEN, linewidth=1.4,
            label='Open-Loop Planned Trajectory',
            zorder=6,
            solid_capstyle='round', solid_joinstyle='round')

    mark_start_end(ax, env['start'], env['goal'])

    # ── 图例：置于右下角（障碍物区域，不遮挡轨迹）────────────────────────
    # 手动排列图例顺序：从欠阻尼到过阻尼，最后是开环
    handles, labels = ax.get_legend_handles_labels()
    # 当前顺序：7.20, 3.60, 0.05, 0.45, 1.80, Open-Loop
    # 期望顺序：0.05, 0.45, 1.80, 3.60, 7.20, Open-Loop
    order = [2, 3, 4, 1, 0, 5]
    handles = [handles[i] for i in order]
    labels  = [labels[i]  for i in order]

    leg = ax.legend(
        handles, labels,
        loc='lower right',
        fontsize=9.5,
        framealpha=0.92,
        edgecolor='#CCCCCC',
        handlelength=2.0,
    )
    leg.get_frame().set_linewidth(0.8)

    save(fig, 'plot_D_kyaw_compare_en.png')


if __name__ == '__main__':
    print('=== 生成图D：K_yaw 参数敏感性对比（EN，有图例）===')
    plot_D()
    print('完成。')

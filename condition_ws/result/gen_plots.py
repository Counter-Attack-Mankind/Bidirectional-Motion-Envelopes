"""
论文配图生成脚本
每种组合：lang(cn/en) × legend(有/无) → 共 3×2×2 = 12 张 PNG
  A: 开环（细实线在上）vs 闭环（粗实线在下），靠粗细差异区分
  B: v1 LIOM vs v2 ECC 规划路径对比
  C: v1 vs v2 速度-时间对比
"""

import csv
import json
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.font_manager as fm

# ─── 路径 ────────────────────────────────────────────────────────────────────
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG  = os.path.join(BASE, 'src', 'condition_planner', 'config')
RES  = os.path.join(BASE, 'result')

# ─── 字体 ────────────────────────────────────────────────────────────────────
CN_FONT = None
for _cand in ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Noto Sans CJK JP', 'SimHei']:
    if _cand in [f.name for f in fm.fontManager.ttflist]:
        CN_FONT = _cand
        break
print(f'Chinese font: {CN_FONT}')

# ─── 颜色 / 线宽 ─────────────────────────────────────────────────────────────
C_OBS  = '#DEDEDE'
E_OBS  = '#888888'
C_V1   = '#2E86C1'   # v1 / 开环（蓝）
C_V2   = '#E67E22'   # v2 / ECC / 闭环（橙）
C_GRID = '#EEEEEE'
LW_OBS = 0.8

# ─── 数据加载 ─────────────────────────────────────────────────────────────────
def load_env():
    with open(os.path.join(CFG, 'env.json')) as f:
        return json.load(f)

def load_traj(fname):
    rows = []
    with open(os.path.join(CFG, fname)) as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) for k, v in r.items()})
    return rows

def load_tracking(fname):
    """加载有效跟踪帧。
    条件：t > 0，且（ref_v > 0 活跃跟踪中）或（车辆已离开起始点 >5cm，捕获末端减速到停止帧）。
    """
    START_X, START_Y = 0.25192, 1.51437
    rows = []
    with open(os.path.join(RES, fname)) as f:
        for r in csv.DictReader(f):
            try:
                t     = float(r['time'])
                ref_v = float(r['ref_v'])
                ox    = float(r['obj_x'])
                oy    = float(r['obj_y'])
                dist  = np.sqrt((ox - START_X)**2 + (oy - START_Y)**2)
                if t > 0 and (ref_v > 0.001 or dist > 0.05):
                    rows.append(r)
            except (ValueError, KeyError):
                pass
    return rows


# ─── 绘图辅助 ─────────────────────────────────────────────────────────────────
def draw_obstacles(ax, env):
    """障碍物2（索引1）和5（索引4）合并为L形多边形，消除内部共享边。
    障碍物不画轮廓线（edgecolor='none'），只有灰色填充。
    """
    skip = {1, 4}
    for i, obs in enumerate(env['obstacles']):
        if i in skip:
            continue
        xs = [p[0] for p in obs]; ys = [p[1] for p in obs]
        ax.add_patch(patches.FancyBboxPatch(
            (min(xs), min(ys)), max(xs)-min(xs), max(ys)-min(ys),
            boxstyle="square,pad=0",
            facecolor=C_OBS, edgecolor='none', zorder=2))

    # 合并多边形：障碍物2 + 障碍物5
    ax.add_patch(patches.Polygon(
        [(1.30, 0.55), (1.70, 0.55), (1.70, 1.80), (1.30, 1.80),
         (1.30, 1.10), (1.20, 1.10), (1.20, 0.85), (1.30, 0.85)],
        closed=True,
        facecolor=C_OBS, edgecolor='none', zorder=2))

def setup_field_ax(ax, env):
    """场地轴：障碍物、坐标范围，无网格线，保留坐标轴框线。"""
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
    ax.plot(start[0], start[1], marker='>', ms=9,  color='#27AE60',
            zorder=6, clip_on=False)
    ax.plot(end[0],   end[1],   marker='*', ms=12, color='#C0392B',
            zorder=6, clip_on=False)

def set_cn_font(ax):
    if CN_FONT is None:
        return
    prop = fm.FontProperties(family=CN_FONT)
    targets = (ax.texts + [ax.xaxis.label, ax.yaxis.label] +
               ax.get_xticklabels() + ax.get_yticklabels())
    for t in targets:
        t.set_fontproperties(prop)
    leg = ax.get_legend()
    if leg:
        for t in leg.get_texts():
            t.set_fontproperties(prop)

def save(fig, name):
    path = os.path.join(RES, name)
    fig.savefig(path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f'  Saved: {path}')
    plt.close(fig)


# ═══════════════════════════════════════════════════════════════════════════════
# 图 A：开环 vs 闭环（v2 ECC）
# 闭环（橙，粗 lw=2.8）画在下层，开环（蓝，细 lw=1.2）压在上层。
# 重合处显蓝色；偏差处橙色边缘从蓝线两侧透出。
# ═══════════════════════════════════════════════════════════════════════════════
def plot_A(lang, show_legend=True):
    env      = load_env()
    traj_v2  = load_traj('trajectory_v2.csv')
    tracking = load_tracking('pp_tracking_log_v2.csv')

    ox = [r['x']             for r in traj_v2]
    oy = [r['y']             for r in traj_v2]
    cx = [float(r['obj_x'])  for r in tracking]
    cy = [float(r['obj_y'])  for r in tracking]

    if lang == 'cn':
        lbl_open   = '开环规划轨迹'
        lbl_closed = '闭环跟踪轨迹'
    else:
        lbl_open   = 'Open-Loop Planned Trajectory'
        lbl_closed = 'Closed-Loop Tracking Trajectory'

    fig, ax = plt.subplots(figsize=(10, 6.5))
    fig.patch.set_facecolor('white')
    setup_field_ax(ax, env)

    # 闭环：粗实线，画在下层
    ax.plot(cx, cy, color=C_V2, linewidth=2.8,
            label=lbl_closed if show_legend else '_nolegend_',
            zorder=4, solid_capstyle='round', solid_joinstyle='round')
    # 开环：细实线，压在上层
    ax.plot(ox, oy, color=C_V1, linewidth=1.2,
            label=lbl_open if show_legend else '_nolegend_',
            zorder=5, solid_capstyle='round', solid_joinstyle='round')

    mark_start_end(ax, env['start'], env['goal'])

    if show_legend:
        ax.legend(loc='upper left', fontsize=10, framealpha=0.9,
                  edgecolor='#CCCCCC')

    if lang == 'cn':
        set_cn_font(ax)

    suffix = f'{lang}' if show_legend else f'{lang}_nolabel'
    save(fig, f'plot_A_open_closed_loop_{suffix}.png')


# ═══════════════════════════════════════════════════════════════════════════════
# 图 B：v1 vs v2 规划路径对比
# ═══════════════════════════════════════════════════════════════════════════════
def plot_B(lang, show_legend=True):
    env     = load_env()
    traj_v1 = load_traj('trajectory_v1.csv')
    traj_v2 = load_traj('trajectory_v2.csv')

    x1 = [r['x'] for r in traj_v1]; y1 = [r['y'] for r in traj_v1]
    x2 = [r['x'] for r in traj_v2]; y2 = [r['y'] for r in traj_v2]

    if lang == 'cn':
        lbl_v1  = 'V1 LIOM 规划路径'
        lbl_v2  = 'V2 ECC 规划路径'
        ann_txt = '最大偏差 ≈ 27 cm'
    else:
        lbl_v1  = 'V1 LIOM Planned Path'
        lbl_v2  = 'V2 ECC Planned Path'
        ann_txt = 'Max deviation ≈ 27 cm'

    fig, ax = plt.subplots(figsize=(10, 6.5))
    fig.patch.set_facecolor('white')
    setup_field_ax(ax, env)

    ax.plot(x1, y1, color=C_V1, linewidth=1.8,
            label=lbl_v1 if show_legend else '_nolegend_', zorder=5)
    ax.plot(x2, y2, color=C_V2, linewidth=1.8,
            label=lbl_v2 if show_legend else '_nolegend_', zorder=5)

    mark_start_end(ax, env['start'], env['goal'])

    if show_legend:
        idx   = 66
        mid_x = (x1[idx] + x2[idx]) / 2
        mid_y = (y1[idx] + y2[idx]) / 2
        ax.annotate(ann_txt,
                    xy=(mid_x, mid_y),
                    xytext=(mid_x + 0.35, mid_y - 0.18),
                    fontsize=9.5, color='#555555',
                    arrowprops=dict(arrowstyle='->', color='#555555', lw=0.9),
                    zorder=7)
        ax.legend(loc='upper left', fontsize=10, framealpha=0.9,
                  edgecolor='#CCCCCC')

    if lang == 'cn':
        set_cn_font(ax)

    suffix = f'{lang}' if show_legend else f'{lang}_nolabel'
    save(fig, f'plot_B_path_compare_{suffix}.png')


# ═══════════════════════════════════════════════════════════════════════════════
# 图 C：速度-时间对比（去掉上/右框线）
# ═══════════════════════════════════════════════════════════════════════════════
def plot_C(lang, show_legend=True):
    traj_v1 = load_traj('trajectory_v1.csv')
    traj_v2 = load_traj('trajectory_v2.csv')

    t1 = [r['t'] for r in traj_v1]; v1 = [r['v'] for r in traj_v1]
    t2 = [r['t'] for r in traj_v2]; v2 = [r['v'] for r in traj_v2]
    v_limit = max(v2)

    if lang == 'cn':
        lbl_v1    = 'V1 LIOM'
        lbl_v2    = 'V2 ECC'
        lbl_limit = f'ECC 速度上限 {v_limit:.3f} m/s'
        xlabel    = '时间 $t$ (s)'
        ylabel    = '速度 $v$ (m/s)'
    else:
        lbl_v1    = 'V1 LIOM'
        lbl_v2    = 'V2 ECC'
        lbl_limit = f'ECC speed limit {v_limit:.3f} m/s'
        xlabel    = 'Time $t$ (s)'
        ylabel    = 'Velocity $v$ (m/s)'

    fig, ax = plt.subplots(figsize=(8, 4.5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.grid(False)

    ax.plot(t1, v1, color=C_V1, linewidth=1.8,
            label=lbl_v1 if show_legend else '_nolegend_', zorder=4)
    ax.plot(t2, v2, color=C_V2, linewidth=1.8,
            label=lbl_v2 if show_legend else '_nolegend_', zorder=4)
    ax.axhline(v_limit, color=C_V2, linewidth=1.0, linestyle='--', alpha=0.6,
               label=lbl_limit if show_legend else '_nolegend_', zorder=3)

    ax.set_xlim(0, max(t1[-1], t2[-1]) * 1.02)
    ax.set_ylim(-0.01, 0.28)
    ax.set_xlabel(xlabel, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.tick_params(labelsize=10)

    if show_legend:
        ax.legend(fontsize=10, framealpha=0.9, edgecolor='#CCCCCC')

    if lang == 'cn':
        set_cn_font(ax)

    suffix = f'{lang}' if show_legend else f'{lang}_nolabel'
    save(fig, f'plot_C_velocity_compare_{suffix}.png')


# ─── 主程序 ───────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=== 生成论文配图 ===')
    for lang in ('cn', 'en'):
        for show_legend in (True, False):
            tag = '有图例' if show_legend else '无图例'
            print(f'\n--- lang={lang}, {tag} ---')
            plot_A(lang, show_legend)
            plot_B(lang, show_legend)
            plot_C(lang, show_legend)
    print('\n完成！共生成12张图。')

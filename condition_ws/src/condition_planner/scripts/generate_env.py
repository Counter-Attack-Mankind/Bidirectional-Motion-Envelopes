#!/usr/bin/env python3
"""生成 env.json：混合方向Z形布局, 4个90°弯
3主块 + 2角落块 = 5 AABB, 左右适度不对称
路径: →右 ↓下(弯1) →右(弯2) ↑上(弯3) →右(弯4) → GOAL
每个90°弯扫掠R=0.393m, 拐角区预留≥R+W/2=0.49m
"""
import numpy as np
import json, os

FIELD_W = 2.80
FIELD_H = 1.80
R_MIN = 0.143 / np.tan(0.3491)  # 0.393m
W = 0.191  # 车宽

# ============ 3个主块(不对称但均衡) ============
obstacles = [
    # obs1: 左块(合并原obs1+obs4, 0.50×1.30) - 封堵左侧从底到中
    [[0.00, 0.00], [0.50, 0.00], [0.50, 1.30], [0.00, 1.30]],

    # obs2: 中块(0.40×1.25) - 封堵中上, 迫使Z形绕行
    [[1.30, 0.55], [1.70, 0.55], [1.70, 1.80], [1.30, 1.80]],

    # obs3: 右块(0.55×1.05) - 封堵右下
    [[2.25, 0.00], [2.80, 0.00], [2.80, 1.05], [2.25, 1.05]],

    # obs4: 右上角落(小)
    [[2.25, 1.62], [2.80, 1.62], [2.80, 1.80], [2.25, 1.80]],

    # obs5: obs2左侧小凸块(装饰, 不影响通行)
    [[1.20, 0.85], [1.30, 0.85], [1.30, 1.10], [1.20, 1.10]],
]

# 起终点: 4个90°弯后方向不变(都朝右θ=0)
START = [0.25, 1.52, 0.0]   # 顶部通道左端, 朝右
GOAL  = [2.50, 1.35, 0.0]   # 右侧出口, 朝右

# ============ 生成 ============
def densify_border(w, h, res=0.01):
    pts = []
    for x in np.arange(0, w, res): pts.append([round(x,6), 0.0])
    for y in np.arange(0, h, res): pts.append([round(w,6), round(y,6)])
    for x in np.arange(w, 0, -res): pts.append([round(x,6), round(h,6)])
    for y in np.arange(h, 0, -res): pts.append([0.0, round(y,6)])
    pts.append([0.0, 0.0])
    return pts

border = densify_border(FIELD_W, FIELD_H)
obs_json = [[[round(p[0],6), round(p[1],6)] for p in obs] for obs in obstacles]

env = {
    "border": border, "obstacles": obs_json, "regions": [], "road": [],
    "start": [round(START[0],6), round(START[1],6), round(START[2],6)],
    "goal": [round(GOAL[0],6), round(GOAL[1],6), round(GOAL[2],6)],
}

out_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
os.makedirs(out_dir, exist_ok=True)
out_path = os.path.join(out_dir, 'env.json')
with open(out_path, 'w') as f:
    json.dump(env, f, separators=(',', ':'))

print(f"生成: {os.path.abspath(out_path)}")
print(f"  5 obstacles, start={env['start']}, goal={env['goal']}")

# 验证
print(f"\n=== 通道 ===")
print(f"  C1(顶→右): y=1.30~1.80, x=0~1.30  h=0.50m")
print(f"  C2(左↓下): x=0.50~1.30, y=0.55~1.30  w=0.80m")
print(f"  C3(底→右): y=0~0.55, x=0.50~2.25  h=0.55m")
print(f"  C4(右↑上): x=1.70~2.25, y=0~1.62  w=0.55m")
print(f"  C5(出口→): y=1.05~1.62, x=1.70~2.80")

print(f"\n=== 弯道扫掠验证(R={R_MIN:.3f}m) ===")
# 弯1: 从x≈0.70开始右转下行
x_after = 0.70 + R_MIN;  print(f"  弯1(→↓): x扫到{x_after:.2f}, 需<obs2左1.30 {'✓' if x_after+W/2<1.30 else '✗'}")
# 弯2: 从x≈1.10开始左转右行
x_after = 1.10 + R_MIN;  print(f"  弯2(↓→): x扫到{x_after:.2f}, 需<obs2左1.30 {'✓' if x_after+W/2<1.30 else '✗'}")
# 弯3: 从x≈1.75开始左转上行
x_after = 1.75 + R_MIN;  print(f"  弯3(→↑): x扫到{x_after:.2f}, 需<obs3左2.25 {'✓' if x_after+W/2<2.25 else '✗'}")
# 弯4: 从x≈2.10开始右转右行
x_after = 2.10 + R_MIN;  print(f"  弯4(↑→): x扫到{x_after:.2f}, 需<场右2.80 {'✓' if x_after+W/2<2.80 else '✗'}")

# 终点验证
fx = GOAL[0] + 0.143 + 0.036  # 车头
print(f"\n  终点车头x={fx:.3f} < 场右2.80 {'✓' if fx<2.80 else '✗'}")
print(f"  终点朝向θ=0(朝右), 与4个90°弯后方向一致 ✓")

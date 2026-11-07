# Condition Branch Constraint Planner

条件分支约束轨迹规划实验系统。

## 编译

```bash
cd ~/Workspace/condition_ws
catkin_make
source devel/setup.bash   # bash
source devel/setup.zsh    # zsh
```

## 仿真规划（版本1：普通LIOM轨迹）

在线计算轨迹（Hybrid A* + LIOM），结果自动保存到 `config/trajectory_v1.csv`。

```bash
roslaunch condition_planner test.launch
```

RViz 会自动打开，显示场景和规划结果。也可以在 RViz 中用 2D Pose Estimate / 2D Nav Goal 覆盖起终点重新规划。

## 实车回放

### 一键启动

```bash
./exp_scripts/run_replay.sh
```

启动 6 个标签页：roscore、定位、底盘、RViz场景、轨迹回放、Pure Pursuit。

### 操作步骤

1. 等所有节点启动完成
2. 在 RViz 中看蓝色起点车身足迹和朝向箭头
3. 把实车对齐到起点位置和朝向
4. 切到「轨迹回放」标签页，按 **Enter** 发送轨迹
5. 车开始跟踪

### 参数修改

编辑 `exp_scripts/run_replay.sh` 顶部：

```bash
CSV="..."       # 轨迹文件路径
TARGET=3        # 车辆ID
VRPN_SERVER="10.1.1.198"  # 定位服务器IP
```

## 场景生成

修改场景后重新生成 `env.json`：

```bash
python3 src/condition_planner/scripts/generate_env.py
```

## 跟踪效果分析

实车跑完后，PP 日志自动保存到 `/tmp/pp_tracking_log.csv`。画图：

```bash
python3 src/condition_planner/scripts/plot_tracking.py
```

输出 `config/tracking_result.png`，包含：XY轨迹对比、横向误差、速度/油门、转向角。

## 可视化搜索节点（调试用）

Hybrid A* 搜索失败时，分析搜索节点分布：

```bash
python3 src/condition_planner/scripts/visualize_hastar_search.py
```

输出 `config/hastar_search_debug.png`。

---

## 跟踪控制器设计与控制理论分析

### 控制器架构：前馈+反馈 2DOF

```
        规划轨迹 (v_ref, κ, θ_ref)
             │
             ▼
    ┌────────────────────────────────┐
    │  纵向: throttle = v_ref + Kp·e_lon    │
    │                                       │
    │  横向: δ = atan(L·κ)                   │
    │          + K_e · e_lat                 │
    │          + K_ψ · e_yaw                 │
    └────────────────────────────────┘
             │
             ▼
          底盘执行
```

**纵向控制：**

```
throttle = v_ref(t) + Kp · e_longitudinal
```

- `v_ref(t)` = 轨迹在当前时刻的规划速度（前馈）
- `e_longitudinal` = 车沿轨迹切线方向的位置误差（反馈）
- Kp = 1.0, 输出限幅 [0, 0.25] m/s（只前进）

**横向控制：**

```
δ = atan(L · κ)  +  K_e · e_lat  +  K_ψ · e_yaw
```

- `κ` = 轨迹曲率，从相邻点中心差分计算
- `e_lat` = 车到最近轨迹点的垂直距离（有符号）
- `e_yaw` = 轨迹航向 − 车辆航向
- K_e = 15, K_ψ = 1.8, 输出限幅 ±0.3491 rad

**控制频率：50 Hz**

### 控制理论分析过程

#### 第一步：建立被控对象模型（自行车模型线性化）

小车运动学用自行车模型描述。定义误差坐标：
- `e_lat`：横向位置误差（垂直于轨迹切线，正=偏左）
- `e_yaw`：航向误差（轨迹航向 − 车辆航向）

线性化后的误差动力学：

```
ė_lat = v · e_yaw                    ... (1) 横向速度 ≈ 车速 × 航向误差
ė_yaw = (v/L) · δ − v · κ            ... (2) 航向角速度 = 转向贡献 − 路径曲率
```

其中 v 为车速，L 为轴距，δ 为前轮转角，κ 为轨迹曲率。

#### 第二步：分析纯反馈控制器（Pure Pursuit）的局限

Pure Pursuit 本质上是对横向误差的比例控制：

```
δ_PP ≈ (2L / d²) · e_lat
```

d 为 lookahead 距离。代入 (2) 并消去 e_yaw，得闭环系统：

```
ë_lat + (v²·K_PP/L) · e_lat = v² · κ     ← 右边是曲率扰动项
```

这是一个 **Type-0 系统**（无积分环节）。根据 **内模原理**，Type-0 系统对阶跃扰动（弯道 κ=常数）有非零稳态误差：

```
e_ss = κ · d² / (2L)
```

代入 κ=2 m⁻¹, d=0.08 m, L=0.143 m：**e_ss = 4.5 cm**。这就是实测弯道误差偏大的理论解释。

#### 第三步：引入前馈消除稳态误差

在控制器中加入曲率前馈项：

```
δ = atan(L · κ) + K_e · e_lat + K_ψ · e_yaw
    ───────────                                  ← 前馈：直接补偿曲率
                  ─────────────────────────────  ← 反馈：修正残差
```

将前馈代入 (2)：

```
ė_yaw = (v/L)·[atan(L·κ) + K_e·e_lat + K_ψ·e_yaw] − v·κ
       ≈ (v/L)·[L·κ + K_e·e_lat + K_ψ·e_yaw] − v·κ     (小角度近似)
       = v·κ + (v·K_e/L)·e_lat + (v·K_ψ/L)·e_yaw − v·κ
       = (v·K_e/L)·e_lat + (v·K_ψ/L)·e_yaw               ← κ 被完全抵消！
```

**前馈消除了曲率扰动**，反馈只处理残差偏差。稳态误差理论上为零。

#### 第四步：闭环稳定性与参数整定

将控制器代入误差动力学，消去 e_yaw 得到标准二阶系统：

```
ë_lat + (v · K_ψ / L) · ė_lat + (v² · K_e / L) · e_lat = 0
```

对照标准形式 `ë + 2ζωn·ė + ωn²·e = 0`：

```
ωn = v · √(K_e / L)          ← 自然频率
ζ  = K_ψ / (2 · √(K_e · L)) ← 阻尼比
```

**参数整定过程：**

| 参数 | 计算 | 结果 | 物理含义 |
|------|------|------|----------|
| K_e = 15 | ωn = 0.2·√(15/0.143) | ωn = 2.05 rad/s | 闭环带宽，决定跟踪速度 |
| K_ψ = 1.8 | ζ = 1.8/(2·√(15×0.143)) | ζ = 0.61 | 阻尼比，0.6~0.8为最优 |

**检查约束：**
- Nyquist 频率 = π × 50Hz = 157 rad/s >> ωn = 2.05 → 离散化稳定 ✓
- 2% 调节时间 ts ≈ 4/(ζ·ωn) = 3.2s → 能跟上弯道变化 ✓
- ζ = 0.61 → 超调约 9%，可接受 ✓

#### 第五步：调参实验验证

| 版本 | K_e | K_ψ | ζ | 直线误差 | 弯道误差 | 问题 |
|------|-----|-----|---|----------|----------|------|
| v1: 纯PP | - | - | - | 1.3cm | 3.2cm | 弯道稳态误差大（Type-0） |
| v2: FF + K_e=5, K_ψ=2 | 5 | 2.0 | 1.18 | 1.28cm | 1.82cm | 过阻尼，响应慢 |
| v3: FF + K_e=15, K_ψ=1.8 | 15 | 1.8 | 0.61 | 1.12cm | 1.30cm | 适当阻尼 |
| **v4: v3 + 50Hz** | 15 | 1.8 | 0.61 | **0.95cm** | **1.27cm** | 离散漂移减半 |

控制频率的影响：每个控制周期内车无控制漂移 = v · dt。从25Hz(0.8cm/步)到50Hz(0.4cm/步)直接降低了误差下限。

### 实测性能

| 指标 | 数值 |
|------|------|
| 直线段横向误差 | mean = 0.95 cm |
| 弯道段横向误差 | mean = 1.27 cm |
| 整体横向误差 | mean = 1.16 cm, max = 2.87 cm |
| 轨迹耗时匹配 | 20.5s / 21.8s (94%) |
| 终点位置误差 | 1.97 cm |

---

## 目录结构

```
condition_ws/
├── exp_scripts/
│   └── run_replay.sh              # 实车回放一键启动
├── src/
│   ├── condition_planner/         # 主规划包
│   │   ├── config/
│   │   │   ├── env.json           # 场景(障碍物+起终点)
│   │   │   ├── planner.yaml       # 车辆参数+规划参数
│   │   │   ├── trajectory_v1.csv  # 版本1轨迹(规划后自动保存)
│   │   │   └── rviz.rviz
│   │   ├── hybrid_astar/          # 混合A*(仅前进)
│   │   ├── trajectory_nlp/        # LIOM优化器(CasADi)
│   │   ├── common/                # 数学库+工具
│   │   ├── scripts/
│   │   │   ├── generate_env.py    # 生成场景
│   │   │   ├── plot_tracking.py   # 画跟踪效果
│   │   │   └── visualize_hastar_search.py
│   │   └── launch/
│   │       ├── test.launch        # 仿真规划
│   │       ├── replay_viz.launch  # 回放可视化
│   │       └── localization.launch
│   ├── sandbox_msgs/              # 消息定义
│   ├── pure_pursuit/              # 轨迹跟踪控制器(2DOF: FF+FB)
│   ├── chassis/                   # 底盘硬件接口(串口)
│   ├── nokov_localization/        # Nokov定位
│   └── vrpn_client_ros/           # VRPN客户端
└── ref/                           # 参考代码(不编译)
```

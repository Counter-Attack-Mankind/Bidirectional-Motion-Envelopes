function InitializeParams()
global params % 声明全局变量，方便在多个函数中共享数据
params.colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39]./255; % 绘图用的颜色库（RGB归一化）

%%  地图环境的参数定义
params.ha.enable_recording = 0; % 是否录制 Hybrid A* 的搜索动画（0关闭，1开启）
params.environment.xmin = 0; % 地图 X 轴最小值 (米)
params.environment.xmax = 30; % 地图 X 轴最大值 (米)
params.environment.ymin = 0; % 地图 Y 轴最小值
params.environment.ymax = 30; % 地图 Y 轴最大值
params.environment.xhorizon = params.environment.xmax - params.environment.xmin; % 场景 X 方向的总跨度
params.environment.yhorizon = params.environment.ymax - params.environment.ymin; % 场景 Y 方向的总跨度

%% 车的几何参数定义
params.vehicle.lf = 0.96;   %定义前悬长度
params.vehicle.lw = 2;    %定义轴距
params.vehicle.LF = params.vehicle.lf + params.vehicle.lw;  % 车头到后轴的总长度
params.vehicle.lr = 0.929;      %定义后悬长度
params.vehicle.lb = 1.92;       %定义车宽
params.vehicle.hlb = params.vehicle.lb / 2;     %定义半车宽,用于计算碰撞边界
params.vehicle.length = params.vehicle.lf + params.vehicle.lw + params.vehicle.lr;  %定义车体总长度

%% 车的动力学参数定义
params.vehicle.v_max = 5;   %速度最大值
params.vehicle.phy_max = 0.7;   %前轴角最大值
params.vehicle.a_max = 0.75;    %速度最大值
params.vehicle.w_max = 0.5;     %车辆角速度最大值
params.vehicle.min_turning_radius = params.vehicle.lw / (tan(params.vehicle.phy_max));      %计算车辆的最小转弯半径，R = Lw/tan(phy)
params.vehicle.max_kappa = 1 / params.vehicle.min_turning_radius;      %车辆的最大曲率，即1/车辆最小转弯半径 kappa = 1/R

%% Hybrid A* 搜索栅格与权重
params.ha.dx = 0.1;
params.ha.dy = 0.1;
params.ha.dtheta = 0.1;

params.ha.nx = ceil(params.environment.xhorizon / params.ha.dx) + 1;
params.ha.ny = ceil(params.environment.yhorizon / params.ha.dy) + 1;
params.ha.ntheta = ceil(2*pi / params.ha.dtheta) + 1;

params.ha.num_phy_ha = 5;
params.ha.simu_unit_duration = 2;

params.ha.penalty_on_phy_change = 0.01;
params.ha.penalty_for_backward = 5;
params.ha.multiplier_on_heuristics = 10;
params.ha.rs_reverse_cost = 5;
params.ha.rs_trigger_distance = 8.0;
params.ha.rs_try_interval = 20;
params.ha.rs_sample_ds = 0.05;

params.ha.max_iter = 20000;
params.ha.max_search_time = 20.0;

params.ha.enable_recording = 0;
params.ha.iter_for_HA = 99000;
params.ha.nfe = 200;

%% 非线性规划问题的参数配置
params.nlp.threshold_rate = 0.6; % 轨迹采样阈值，用于判断何时保留采样点
params.nlp.dt = 0.0001; % NLP 的基础微小时间增量
params.nlp.nfe = 0; % 记录优化后的点数
params.nlp.std_nfe_fixed = -999; % 预留的固定点数标志位
params.nlp.new_nfe_fixed = -999; % 预留的固定点数标志位
params.guiding_path.x = []; % 存储引导路径的占位符
params.guiding_path.y = [];
params.ef.max_dt = 0.25; % 具身足迹的最大时间跨度
params.ef.min_dt = 0.001; % 具身足迹的最小时间跨度
params.ef.temporal_tightening_eta = 0.9;
params.ef.shrink.scale_min = 0;
params.ef.shrink.scale_step = 0.02;
params.ef.shrink.safety_slack = 1e-7;
%% 具身足迹半径与 LIOM 优化设置
params.vehicle.p2r = 0.25 * params.vehicle.length - params.vehicle.lr; % 车身几何中心偏置计算（后部）
params.vehicle.p2f = 0.75 * params.vehicle.length - params.vehicle.lr; % 车身几何中心偏置计算（前部）
params.vehicle.radius = 0.5 * hypot(0.5 * params.vehicle.length, params.vehicle.lb); % 车辆外接圆半径（粗略碰撞检测用）
params.liom.stc.ds_for_init_adjustment = 0.01; % 线性化微调步长
params.liom.stc.ds = 0.1; % 离散化步长
params.liom.stc.smax = 10.0; % 最大路径跨度
params.liom.acceptance_tolerance = 0.0001; % 优化收敛容忍度
params.liom.cost_function_external_penalty_weight = 100000; % 碰撞惩罚权重的系数（很大说明绝对不能碰撞）
params.liom.max_iter = 5; % LIOM 算法内部最大迭代次数
end

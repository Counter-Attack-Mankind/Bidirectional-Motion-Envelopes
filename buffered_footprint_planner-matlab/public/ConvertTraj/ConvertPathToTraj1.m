function [x, y, theta, v, a, phy, w, time] = ConvertPathToTraj()
% LEGACY: historical converter; current runs use ConvertPathToTraj.m.
% ============================================================
% ConvertPathToTraj
% 作用：
%   将 Hybrid A* 输出的离散路径点 (x,y,theta)
%   转换为带时间戳的轨迹 (x,y,theta,v,a,phy,w,time)
%
% 输出含义：
%   x, y, theta : 轨迹上的离散状态点（时间序列）
%   v           : 速度（单位 m/s，符号是否支持倒车取决于 CalculateTimeStamp 实现）
%   a           : 加速度（单位 m/s^2）
%   phy         : 方向盘转角（单位 rad）
%   w           : 方向盘角速度（单位 rad/s）或转角变化率（看你的定义）
%   time        : 每个点对应的时间戳（单位 s）
%
% 注意：
%   这个函数本身只用到了 params.ha.x/y/theta，
%   是否支持倒车（v 为负）关键取决于 CalculateTimeStamp() 的实现。
% ============================================================

global params

% =========================
% [模块 1] 读取 Hybrid A* 路径
% =========================
x = params.ha.x;           % Hybrid A* 路径点 x 序列
y = params.ha.y;           % Hybrid A* 路径点 y 序列
theta = params.ha.theta;   % Hybrid A* 路径点航向角 theta 序列（车辆姿态）

%【新增】读取parmas.ha.v来识别前进倒退
if ~isfield(params.ha,'v') || isempty(params.ha.v)
    warning('params.ha.v 不存在，默认全部前进');
    v0 = ones(size(x));
else
    v0 = params.ha.v;
end

% ============================================================
% [模块 2] 删除"重复/静止点"
% 目的：
%   避免出现相邻点完全相同 (x,y) 的情况，
%   因为后续计算速度/时间戳时，dx=dy=0 可能导致除0、NaN等问题。
%
% 做法：
%   从 ii=2 开始遍历，只保留与前一个点 (x(ii-1),y(ii-1)) 不同的点。
% ============================================================
xx = x(1); yy = y(1); tt = theta(1); vv = v0(1);
for ii = 2 : length(x)
    % 判断当前点是否与前一点完全相同
    if (~((x(ii)==x(ii-1))&&(y(ii)==y(ii-1))))
        xx = [xx, x(ii)];       % 保留该点的 x
        yy = [yy, y(ii)];       % 保留该点的 y
        tt = [tt, theta(ii)];   % 保留该点的 theta
        vv = [vv, v0(ii)];     % 【新增】同步保留该点方向
    end
end

% 用过滤后的序列覆盖原序列
x = xx; y = yy; theta = tt; v0 = vv;

% 如果过滤后只剩一个点，无法形成轨迹（没有相邻点）
if (size(x, 2) == 1)
    disp '[ConvertPathToTraj] Path is a single point!'
    return;
end

num_samples = size(x, 2); % 过滤后路径点数量（列向量长度）

% =========================
% [新增] 获取换向点索引 chg
% =========================
if isfield(params,'change') && ~isempty(params.change)
    % 注意：params.change 是基于原始轨迹的索引；已经去重点了，索引可能不一致
    % 所以更推荐从 v0 重新计算（下一段会覆盖）
end

% 保险起见：直接从去重后的 v0 计算换向点（最可靠）
% 1. 计算换向索引（方向发生变化的位置）
change_idx = find(v0(2:end) ~= v0(1:end-1)) + 1;
params.change = change_idx;

% 2. 输出换向次数
fprintf('换向次数: %d\n', numel(change_idx));

% 3. 输出每个换向点的序号和坐标
for k = 1:numel(change_idx)
    idx = change_idx(k);
    fprintf('去重后，换向点为 #%d: idx=%d, x=%.3f, y=%.3f\n', ...
        k, idx, x(idx), y(idx));
end

% ============================================================
% [模块 3] theta 连续化（unwrap）
% 目的：
%   避免角度跳变。例如从 179° 到 -179° 会产生接近 -358° 的跳变。
%   这会破坏后续的 dtheta/dt 或 dtheta/ds 计算。
%
% 做法：
%   若 theta(ii)-theta(ii-1) > pi  => 当前角度减去 2pi
%   若 theta(ii)-theta(ii-1) < -pi => 当前角度加上 2pi
% ============================================================
for ii = 2 : num_samples
    while (theta(ii) - theta(ii-1) > pi)
        theta(ii) = theta(ii) - 2 * pi;
    end
    while (theta(ii) - theta(ii-1) < -pi)
        theta(ii) = theta(ii) + 2 * pi;
    end
end

% =========================
% [模块 4-改] 由于这里我选用的是全前进段、前进+后退段验证，因此速度匹配采用A-B一次性计算或者A-B-C两段的分段速度匹配
% 目标：换向点 B 速度强制为 0
% =========================

if isempty(change_idx)
    % ---- 无换向：整段同方向（全前进 或 全倒车） ----
    [terminal_time, x1, y1, theta1, v_mag, a_mag] = CalculateTimeStamp(x, y, theta);

    % 整段符号统一：用 v0(1)（去重后第一个点的方向）
    v = abs(v_mag) * sign(v0(1));
    a = a_mag * sign(v0(1));

else
    % ---- 有换向：只取第一个换向点（A-B-C）----
    B = change_idx(1);   % B 点索引（在去重后的 x/y/theta/v0 上）

    % A-B 段（包含 B）
    x_AB = x(1:B);
    y_AB = y(1:B);
    th_AB = theta(1:B);

    % B-C 段（从 B 开始，包含 B，避免几何断裂）
    x_BC = x(B:end);
    y_BC = y(B:end);
    th_BC = theta(B:end);

    % 分别做时间最优速度剖面（都是"幅值"）
    [T1, x1_AB, y1_AB, th1_AB, v1_mag, a1_mag] = CalculateTimeStamp(x_AB, y_AB, th_AB);
    [T2, x1_BC, y1_BC, th1_BC, v2_mag, a2_mag] = CalculateTimeStamp(x_BC, y_BC, th_BC);

    % 给每段加符号（用该段方向）
    sgn1 = sign(v0(1));     if sgn1==0, sgn1=1; end
    sgn2 = sign(v0(B));     if sgn2==0, sgn2=sgn1; end  % B 点方向就是切换后的新方向

    v1 = abs(v1_mag) * sgn1;
    a1 = a1_mag * sgn1;

    v2 = abs(v2_mag) * sgn2;
    a2 = a2_mag * sgn2;

    % ---- 强制换向点速度为 0：拼接时在 B 点插入 0 ----
    % 注意：x1_AB 的末点和 x1_BC 的首点通常都在 B 附近/重合
    % 我们拼接时去掉第二段的第一个点，避免重复点
    x1 = [x1_AB, x1_BC(2:end)];
    y1 = [y1_AB, y1_BC(2:end)];
    theta1 = [th1_AB, th1_BC(2:end)];

    % 速度拼接：让拼接处（AB末点/BC首点）为0
    % 最简单：把 AB 最后一个速度设为 0，BC 第一个速度也设为 0（但 BC 第一个点已被去掉）
    v1(end) = 0;
    v2(1) = 0;

    v = [v1, v2(2:end)];
    a = [a1, a2(2:end)];

    terminal_time = T1 + T2;
end

num_samples2 = length(x1);       % 重采样后点的数量
phy = zeros(1, num_samples2);    % 初始化方向盘转角序列 phy(t)
w   = zeros(1, num_samples2);    % 初始化方向盘角速度/变化率序列 w(t)

% 假设时间均匀分布：dt = 总时间 / (点数-1)
dt = terminal_time / (num_samples2 - 1);

% ============================================================
% [模块 5] 由 theta(t) 和 v(t) 反推转向角 phy(t)
% 依据自行车模型（简化）：
%   theta_dot = v / L * tan(phy)
% => tan(phy) = theta_dot * L / v
% => phy = atan(theta_dot * L / v)
%
% 此处离散近似：
%   theta_dot ≈ (theta1(ii+1) - theta1(ii)) / dt
%
% 所以：
%   phy(ii) = atan( (theta1(ii+1)-theta1(ii)) * L / (dt * v(ii)) )
%
% 重要：
%   - 如果 v(ii)=0 或非常小，会导致除0或巨大值 => NaN/Inf
%   - 最后用 phy_max 做饱和限制
% ============================================================
for ii = 2 : (num_samples2 - 1)
    if abs(v(ii)) < 1e-6
        phy(ii) = 0;   % 换向/低速处，避免除0
    else
        phy(ii) = atan((theta1(ii+1) - theta1(ii)) * params.vehicle.lw / (dt * v(ii)));
    end

    % 方向盘转角饱和限制（物理可行）
    if (phy(ii) > params.vehicle.phy_max)
        phy(ii) = params.vehicle.phy_max;
    elseif (phy(ii) < -params.vehicle.phy_max)
        phy(ii) = -params.vehicle.phy_max;
    end
end

% 将 NaN 置为 0（通常来源于 v=0 或 0/0）
phy(isnan(phy)) = 0;

% ============================================================
% [模块 6] 计算 w(t) —— 转向角变化率
% 这里的 w 定义为：
%   w(ii) = (phy(ii+1) - phy(ii)) / dt
%
% 然后对 w 做饱和（最大方向盘角速度约束）
% ============================================================
for ii = 2 : (num_samples2 - 1)
    w(ii) = (phy(ii+1) - phy(ii)) / dt;

    % 方向盘角速度饱和限制
    if (w(ii) > params.vehicle.w_max)
        w(ii) = params.vehicle.w_max;
    elseif (w(ii) < -params.vehicle.w_max)
        w(ii) = -params.vehicle.w_max;
    end
end

%=================寻找速度匹配后的 换向点=========================%
thr = 0.05;  %加入一个阈值
idxF = find(v >  thr);      % 所有"确定前进"的点
idxR = find(v < -thr);      % 所有"确定倒车"的点
chg = [];

if ~isempty(idxF) && ~isempty(idxR)
    % 找到前进最后一个点 和 倒车第一个点
    iF = idxF(end);
    iR = idxR(1);

    if iF < iR
        % 换向点就在它们之间：取中间点（或者取 abs(v) 最小点）
        seg = iF:iR;
        [~, k0] = min(abs(v(seg)));
        chg = seg(k0);
    end
end

%===========================输出换向点=====================
fprintf('速度匹配后，检测到换向次数: %d\n', ~isempty(chg));
if ~isempty(chg)
    fprintf('换向点 idx=%d, x=%.3f, y=%.3f, v=%.6f\n', ...
        chg, x1(chg), y1(chg), v(chg));
end

%============================输出换向点前500和后500序列号的速度，步长为20=====================================
if ~isempty(chg)

    start_idx = max(1, chg - 100);
    end_idx   = min(length(x1), chg + 100);

    step = 20;

    fprintf('\n===== 换向点附近轨迹 =====\n');

    for ii = start_idx:step:end_idx
        fprintf('序号: %d, x: %.3f, y: %.3f, v: %.3f\n', ...
            ii, x1(ii), y1(ii), v(ii));
    end

    % 确保换向点一定输出
    fprintf('*** 换向点 *** idx=%d, x=%.3f, y=%.3f, v=%.3f\n', ...
        chg, x1(chg), y1(chg), v(chg));

    fprintf('==========================\n\n');

end


% ============================================================
% [模块 7] 具身筛选
% TimeDistribution() 输出最终轨迹：
%   x, y, theta, v, a, phy, w, time
%
% terminal_time 作为总时长输入
% ============================================================
[x, y, theta, v, a, phy, w, time] = TimeDistribution(x1, y1, theta1, v, a, phy, w, terminal_time);

end

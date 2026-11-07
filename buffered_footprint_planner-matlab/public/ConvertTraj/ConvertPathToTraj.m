function [x, y, theta, v, a, phy, w, time] = ConvertPathToTraj()
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
% [新增] 获取"有效换向点"索引
% 规则：
%   若某个方向段长度 < min_length，则不视为真实换向
% =========================
min_length = 0.05;   % 先固定，后续再调

[change_idx, seg_st, seg_ed, seg_dir, seg_len] =  getEffectiveChangeIdx(x, y, v0, min_length);

params.change = change_idx;

% 输出有效换向次数
fprintf('有效换向次数: %d\n', numel(change_idx));

% 输出每一段信息
fprintf('有效方向段信息如下：\n');
for k = 1:length(seg_st)
    if seg_dir(k) > 0
        dir_str = '前进';
    else
        dir_str = '倒车';
    end

    fprintf('段 #%d: [%d -> %d], 方向=%s, 长度=%.3f m\n', ...
        k, seg_st(k), seg_ed(k), dir_str, seg_len(k));
end

% 输出每个有效换向点坐标
for k = 1:numel(change_idx)
    idx = change_idx(k);
    fprintf('有效换向点 #%d: idx=%d, x=%.3f, y=%.3f\n', ...
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
% [模块 4-改] 基于"有效换向点"的多段速度匹配
% 规则：
%   1) 若无换向点，则整段一次性做速度匹配
%   2) 若有多个有效换向点，则按段分别做 CalculateTimeStamp
%   3) 每个真实换向点处速度强制为 0
%   4) 拼接时去掉后一段的首点，避免重复
% =========================

if isempty(change_idx)

    % 单一方向段
    [terminal_time, x1, y1, theta1, v_mag, a_mag] = ...
        CalculateTimeStamp(x, y, theta);

    % 使用方向分段结果，不再重新从原始 v0 推断
    sgn = seg_dir(1);

    v = abs(v_mag) * sgn;
    a = a_mag * sgn;

else
    % ============================================================
    % 多段切分
    % 例如：
    %   change_idx = [B, C]
    % 则切分点 cut_idx = [1, B, C, N]
    % 段1: 1 ~ B
    % 段2: B ~ C
    % 段3: C ~ N
    % ============================================================
    N = length(x);
    cut_idx = [1, change_idx, N];

    % 初始化拼接结果
    x1 = [];
    y1 = [];
    theta1 = [];
    v = [];
    a = [];
    terminal_time = 0;

    nSeg = length(cut_idx) - 1;

    for s = 1:nSeg
        st = cut_idx(s);
        ed = cut_idx(s+1);

        % 防止异常索引
        if ed <= st
            continue;
        end

        % 取当前段几何
        x_seg = x(st:ed);
        y_seg = y(st:ed);
        th_seg = theta(st:ed);

        % 当前段做时间最优速度匹配（输出为幅值）
        [Ts, x1_seg, y1_seg, th1_seg, v_mag_seg, a_mag_seg] = CalculateTimeStamp(x_seg, y_seg, th_seg);

        % 当前段方向直接采用 getEffectiveChangeIdx 的分段结果
        sgn = seg_dir(s);

        % 加符号
        v_seg = abs(v_mag_seg) * sgn;
        a_seg = a_mag_seg * sgn;

        % ========================================================
        % 真实换向段边界速度置 0
        % 第1段末点是换向点 -> 末速度置0
        % 中间段首末都是换向点 -> 首末都置0
        % 最后一段首点是换向点 -> 首速度置0
        % ========================================================
        if s > 1
            v_seg(1) = 0;
        end

        if s < nSeg
            v_seg(end) = 0;
        end

        % 加速度边界简单同步处理，避免边界突跳太大
        if length(a_seg) >= 1
            if s > 1
                a_seg(1) = 0;
            end
            if s < nSeg
                a_seg(end) = 0;
            end
        end

        % ========================================================
        % 拼接：
        % 第一段完整保留；
        % 后续段去掉首点，避免与前一段换向点重复
        % ========================================================
        if isempty(x1)
            x1 = x1_seg;
            y1 = y1_seg;
            theta1 = th1_seg;
            v = v_seg;
            a = a_seg;
        else
            x1 = [x1, x1_seg(2:end)];
            y1 = [y1, y1_seg(2:end)];
            theta1 = [theta1, th1_seg(2:end)];
            v = [v, v_seg(2:end)];
            a = [a, a_seg(2:end)];
        end

        terminal_time = terminal_time + Ts;
    end
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
% ============================================================
% 检测速度匹配后的真实换向
% ============================================================
thr = 0.05;

v_sign = zeros(size(v));
v_sign(v >  thr) = 1;
v_sign(v < -thr) = -1;

chg = [];

last_nonzero_idx = [];
last_nonzero_sign = 0;

for ii = 1:length(v_sign)

    % 换向点附近 v=0，不直接作为方向
    if v_sign(ii) == 0
        continue;
    end

    if isempty(last_nonzero_idx)
        last_nonzero_idx = ii;
        last_nonzero_sign = v_sign(ii);
        continue;
    end

    % 两个相邻非零运动段方向不同
    if v_sign(ii) ~= last_nonzero_sign

        % 在两个非零方向点之间找 |v| 最小的位置，
        % 即停车/换向配置点。
        search_range = last_nonzero_idx:ii;
        [~, local_idx] = min(abs(v(search_range)));

        cusp_idx = search_range(local_idx);

        chg(end+1) = cusp_idx; %#ok<AGROW>
    end

    last_nonzero_idx = ii;
    last_nonzero_sign = v_sign(ii);
end

chg = unique(chg, 'stable');

fprintf('速度匹配后，检测到换向次数: %d\n', numel(chg));

for k = 1:numel(chg)
    idx = chg(k);

    fprintf('换向点 #%d: idx=%d, x=%.3f, y=%.3f, v=%.6f\n', k, idx, x1(idx), y1(idx), v(idx));
end

%============================输出换向点前500和后500序列号的速度，步长为20=====================================
if ~isempty(chg)

    step = 20;

    for kk = 1:numel(chg)

        cusp_idx = chg(kk);

        start_idx = max(1, cusp_idx - 100);
        end_idx   = min(length(x1), cusp_idx + 100);

        fprintf('\n===== 换向点 #%d 附近轨迹 =====\n', kk);

        for ii = start_idx:step:end_idx
            fprintf('序号: %d, x: %.3f, y: %.3f, v: %.3f\n', ii, x1(ii), y1(ii), v(ii));
        end

        fprintf('*** 换向点 #%d *** idx=%d, x=%.3f, y=%.3f, v=%.6f\n', kk, cusp_idx, x1(cusp_idx), y1(cusp_idx), v(cusp_idx));

        fprintf('==========================\n\n');
    end
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
function [x1, y1, theta1, v, acc, phy, w, time] = TimeDistribution(x1, y1, theta1, v, acc, phy, w, terminal_time)
%TIMEDISTRIBUTION
% 基于缓冲足迹理论有效性条件的初始时间网格构造。
%
%
% 核心原则：
%   1) 稠密参考轨迹仅作为候选配置点集合；
%   2) 起点、终点以及前进/倒退换向点必须保留；
%   3) 每个候选 interval 满足 [min_dt, max_dt]；
%   4) 按照当前配置点状态计算
%
%          kappa_k = tan(phi_k) / Lw
%          s_k     = v_k * Delta_t_k
%
%   5) 使用与 LSE 模型一致的平滑 splus / sminus；
%   6) 使用 eta 收紧后的 EF validity conditions
%      （论文 Eq. (28a)-(28e)）进行候选区间筛选；
%   7) 当继续扩展 interval 首次违反 tightened validity conditions
%      时，保留最后一个 admissible candidate；
%   8) 障碍物碰撞不参与本函数的配置点选择。
%
% 注意：
%   geometric feasibility enhancement 属于后续独立步骤，
%   由 CheckInitialEFCollision / ShrinkWrittenInitialGuessEF 完成。
%
% 输入：
%   x1, y1, theta1  - 稠密参考轨迹
%   v, acc, phy, w  - 稠密参考状态/控制序列
%   terminal_time   - 稠密参考轨迹总时间
%
% 输出：
%   x1, y1, theta1  - 筛选后的 NLP 配置点
%   v, acc, phy, w  - 对应节点变量
%   time            - 区间 dt，最后补 0
%
% -------------------------------------------------------------------------

global params


%% ========================================================================
% Module 1: 输入标准化
% ========================================================================

x1     = x1(:).';
y1     = y1(:).';
theta1 = theta1(:).';
v      = v(:).';
acc    = acc(:).';
phy    = phy(:).';
w      = w(:).';

dense_n = numel(x1);
base_dt = terminal_time / (dense_n - 1);


%% ========================================================================
% Module 2: 时间边界与 tightening factor
% ========================================================================

min_dt = params.ef.min_dt;
max_dt = params.ef.max_dt;
eta = params.ef.temporal_tightening_eta;

% 将 dt 上下界转换为稠密网格索引跨度
min_step = max(1, ceil((min_dt - 1e-12) / base_dt));
max_step = floor((max_dt + 1e-12) / base_dt);

%% ========================================================================
% Module 3: 强制保留前进/倒退换向点
% ========================================================================
% ConvertPathToTraj 已经将真实换向点记录在
% params.ef.dense_change_idx 中。
%
% 起点和终点由本函数自然强制保留。

if isfield(params.ef, 'dense_change_idx') && ~isempty(params.ef.dense_change_idx)

    mandatory_idx = params.ef.dense_change_idx(:).';
    % 起点和终点无需重复放入 mandatory_idx
    mandatory_idx = mandatory_idx(mandatory_idx > 1 & mandatory_idx < dense_n);
    mandatory_idx = unique(mandatory_idx, 'stable');
else
    mandatory_idx = [];
end


%% ========================================================================
% Module 4: 初始化 temporal mesh
% ========================================================================

selected      = 1;
interval_time = [];
cur_idx = 1;


%% ========================================================================
% Module 5: 按论文 Eq. (28) 贪心构造初始时间网格
% ========================================================================
%
% 从当前 retained point 开始，依次向后检查候选点。
%
% 对每个候选 interval：
%
%   Delta_t_k = (cand_idx-cur_idx) * base_dt
%   kappa_k   = tan(phi_k) / Lw
%   s_k       = v_k * Delta_t_k
%
% 然后使用 LSE-smoothed splus/sminus 检查 tightened validity
% conditions。
%
% 一旦继续扩大 interval 首次违反条件，就保留此前最后一个
% admissible candidate。
%
% 障碍物位置完全不参与本过程。
% ========================================================================

while cur_idx < dense_n

    % --------------------------------------------------------------------
    % 找到下一个必须保留的点
    % --------------------------------------------------------------------
    next_mandatory = mandatory_idx(find(mandatory_idx > cur_idx, 1, 'first'));

    if isempty(next_mandatory)
        next_mandatory = dense_n;
    end

    step_to_boundary = next_mandatory - cur_idx;


    % --------------------------------------------------------------------
    % 当前 interval 能够搜索到的最大候选跨度
    % --------------------------------------------------------------------
    upper_step = min(max_step, step_to_boundary);

    % 构造候选 step。
    %
    % 除非候选点本身就是 mandatory point，否则必须确保候选点之后
    % 到 mandatory point 至少还剩 min_step，避免制造一个非法的
    % 短尾 interval。
    candidate_steps = [];

    for step = min_step:upper_step

        remaining = step_to_boundary - step;

        if remaining == 0 || remaining >= min_step
            candidate_steps(end+1) = step; %#ok<AGROW>
        end
    end

    if isempty(candidate_steps)
        error(['TimeDistribution: no candidate step can satisfy the ', ...
               'time-boundary structure at cur_idx=%d.'], cur_idx);
    end


    % --------------------------------------------------------------------
    % 按顺序检查候选 interval
    % --------------------------------------------------------------------
    last_valid_step = [];

    for kk = 1:numel(candidate_steps)

        step = candidate_steps(kk);
        cand_idx = cur_idx + step;

        is_valid = isIntervalValid( ...
            cur_idx, cand_idx, base_dt, v, phy, eta);

        if is_valid
            last_valid_step = step;
        else
            % 严格按照论文：
            % 第一次继续扩展失败后停止，并选择此前最后一个合法候选点。
            break;
        end
    end


    % --------------------------------------------------------------------
    % 最短候选 interval 都不满足 tightened validity conditions
    % --------------------------------------------------------------------
    if isempty(last_valid_step)

        error(['TimeDistribution: no admissible interval exists at ', ...
               'cur_idx=%d. Even the shortest candidate violates ', ...
               'the tightened EF validity conditions.'], cur_idx);
    end


    % --------------------------------------------------------------------
    % 直接采用最后一个 admissible candidate
    %
    % 注意：
    % 不再执行 last_valid_step * 0.9。
    % eta 已经直接作用于 Eq. (28) 的理论边界。
    % --------------------------------------------------------------------
    chosen_step = last_valid_step;
    next_idx = cur_idx + chosen_step;


    % --------------------------------------------------------------------
    % 保存 interval
    % --------------------------------------------------------------------
    selected(end+1) = next_idx; %#ok<AGROW>

    interval_time(end+1) = ...
        chosen_step * base_dt; %#ok<AGROW>

    cur_idx = next_idx;
end


%% ========================================================================
% Module 6: 执行降采样
% ========================================================================

selected = unique(selected, 'stable');

% 理论上 while 循环必须以终点结束
if selected(end) ~= dense_n
    error('TimeDistribution: terminal point was not retained.');
end

params.nfe = numel(selected);

x1     = x1(selected);
y1     = y1(selected);
theta1 = theta1(selected);
v      = v(selected);
acc    = acc(selected);
phy    = phy(selected);
w      = w(selected);

% NLP 约定：
% 前 Nfe-1 个元素表示 interval dt，
% 最后补一个 0。
time = [interval_time, 0];


%% ========================================================================
% Module 7: 最终时间边界检查
% ========================================================================

validateTimeBounds(interval_time, min_dt, max_dt);


%% ========================================================================
% Module 8: 缓存最终 temporal mesh
% ========================================================================

params.ef.x     = x1(:);
params.ef.y     = y1(:);
params.ef.theta = theta1(:);

params.ef.v   = v(:);
params.ef.acc = acc(:);
params.ef.phy = phy(:);
params.ef.w   = w(:);

params.ef.time = time(:);

% 保存 temporal enhancement 参数，便于实验记录
params.ef.temporal_tightening_eta = eta;


%% ========================================================================
% Module 9: 检测筛选后轨迹中的换向
% ========================================================================

params.ef.change_final = detectDirectionSwitch(v);


%% ========================================================================
% Module 10: 调试输出
% ========================================================================

fprintf('\n========== Temporal Feasibility Enhancement ==========\n');
fprintf('dense point count          : %d\n', dense_n);
fprintf('selected point count       : %d\n', params.nfe);
fprintf('dense base dt              : %.6e\n', base_dt);
fprintf('selected interval dt range : %.6e / %.6e\n', ...
    min(interval_time), max(interval_time));
fprintf('temporal tightening eta    : %.6f\n', eta);
fprintf('direction switches         : %d\n', ...
    numel(params.ef.change_final));
fprintf('======================================================\n\n');

end


%% ========================================================================
% Local Function 1
% 检查候选 interval 是否满足 tightened EF validity conditions
% ========================================================================
function is_valid = isIntervalValid(cur_idx, cand_idx, base_dt, v, phy, eta)
global params
tol = 1e-12;
% ------------------------------------------------------------------------
% 1. 防止 interval 跨越前进/倒退运动方向
%
% 正常情况下 mandatory cusp 已经避免此问题；
% 这里保留为一致性检查。
% ------------------------------------------------------------------------

interval_v = v(cur_idx:cand_idx);

has_forward = any(interval_v >  1e-6);
has_reverse = any(interval_v < -1e-6);

if has_forward && has_reverse
    is_valid = false;
    return;
end

% ------------------------------------------------------------------------
% 2. 论文 Eq. (4)
%
%      kappa_k = tan(phi_k) / Lw
%      s_k     = Delta_t_k * v_k
%
% 必须使用当前 retained collocation point k 的状态，
% 不再寻找 interval 内第一个非零速度点。
% ------------------------------------------------------------------------
delta_t = (cand_idx - cur_idx) * base_dt;
kappa_k = tan(phy(cur_idx)) / params.vehicle.lw;
s_k = v(cur_idx) * delta_t;
abs_kappa = abs(kappa_k);

% ------------------------------------------------------------------------
% 3. 与正式 LSE 模型一致的 forward/reverse travel distance
% ------------------------------------------------------------------------
splus  = smoothPlus(s_k);
sminus = smoothPlus(-s_k);
% ------------------------------------------------------------------------
% 4. 论文 Eq. (28a)-(28e)
%
% cond_* = true 表示 tightened validity condition 被违反。
% ------------------------------------------------------------------------

% Eq. (28a)
lhs_arc = abs_kappa * (splus + sminus);
rhs_arc = eta * (pi / 2);
cond_arc = lhs_arc > rhs_arc + tol;

% Eq. (28b)
lhs_f1 =(1 + params.vehicle.hlb * abs_kappa) * tan(abs_kappa * splus);
rhs_f1 = eta * params.vehicle.lr * abs_kappa;
cond_f1 = lhs_f1 > rhs_f1 + tol;

% Eq. (28c)
lhs_f2 = abs_kappa * params.vehicle.LF * tan(abs_kappa * splus);
rhs_f2 = eta * (1 + params.vehicle.hlb * abs_kappa);
cond_f2 = lhs_f2 > rhs_f2 + tol;

% Eq. (28d)
lhs_r1 = (1 + params.vehicle.hlb * abs_kappa) * tan(abs_kappa * sminus);
rhs_r1 = eta * params.vehicle.LF * abs_kappa;
cond_r1 = lhs_r1 > rhs_r1 + tol;

% Eq. (28e)
lhs_r2 = abs_kappa * params.vehicle.lr * tan(abs_kappa * sminus);
rhs_r2 = eta * (1 + params.vehicle.hlb * abs_kappa);
cond_r2 = lhs_r2 > rhs_r2 + tol;


% ------------------------------------------------------------------------
% 5. 综合判断
%
% 不包含任何 obstacle collision test。
% ------------------------------------------------------------------------

is_valid = ~(cond_arc || cond_f1  || cond_f2  || cond_r1  || cond_r2);
end


%% ========================================================================
% Local Function 2
% 稳定计算 LSE smooth max(0,x)
% ========================================================================
function val = smoothPlus(x)

global params

if isfield(params, 'nlp') && ...
        isfield(params.nlp, 'alpha') && ...
        ~isempty(params.nlp.alpha)

    alpha = params.nlp.alpha;

else
    alpha = 60;
end

if ~isfinite(alpha) || alpha <= 0
    error('TimeDistribution: LSE alpha must be positive.');
end

% stable:
%
% (1/alpha) log(exp(0) + exp(alpha*x))
%
% = max(0,x)
% + log(exp(-alpha*m) + exp(alpha*(x-m))) / alpha

m = max(0, x);

val = m + ...
    log(exp(-alpha * m) + exp(alpha * (x - m))) / alpha;

end


%% ========================================================================
% Local Function 3
% 校验最终 interval dt
% ========================================================================
function validateTimeBounds(interval_time, min_dt, max_dt)

tol = 1e-10;

if isempty(interval_time)
    error('TimeDistribution: no interval was generated.');
end

if any(~isfinite(interval_time)) || any(interval_time <= 0)
    error('TimeDistribution: invalid non-positive or non-finite dt.');
end

if any(interval_time < min_dt - tol) || ...
        any(interval_time > max_dt + tol)

    error(['TimeDistribution produced dt outside bounds: ', ...
           'actual min/max = %.12g / %.12g, ', ...
           'required min/max = %.12g / %.12g.'], ...
           min(interval_time), max(interval_time), ...
           min_dt, max_dt);
end

end


%% ========================================================================
% Local Function 4
% 检测最终配置点中的前进/倒退切换
% ========================================================================
function chg_ef = detectDirectionSwitch(v)

thr = 0.05;

v_sign = zeros(size(v));

v_sign(v >  thr) =  1;
v_sign(v < -thr) = -1;

chg_ef = [];

last_nonzero_sign = 0;

for i = 1:numel(v_sign)

    if v_sign(i) == 0
        continue;
    end

    if last_nonzero_sign == 0
        last_nonzero_sign = v_sign(i);
        continue;
    end

    if v_sign(i) ~= last_nonzero_sign
        chg_ef(end+1) = i; %#ok<AGROW>
    end

    last_nonzero_sign = v_sign(i);
end

end
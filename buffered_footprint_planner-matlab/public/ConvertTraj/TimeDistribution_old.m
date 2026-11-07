function [x1, y1, theta1, v, acc, phy, w, time] = TimeDistribution_old(x1, y1, theta1, v, acc, phy, w, terminal_time)
% LEGACY: historical temporal-mesh implementation; not used by RunMe1/RunMe2.
%TIMEDISTRIBUTION  基于 EF 可行性的轨迹降采样与时间分配
%
% 功能：
%   从稠密初始轨迹中筛选 NLP1 使用的离散配置点。
%   筛选原则是：相邻两个配置点之间的时间间隔满足 NLP1 的 dt 上下界，
%   同时对应的 EF 扩展包络盒满足避障约束和 EF 前提约束。
%
% 输入：
%   x1, y1, theta1  - 稠密轨迹位姿
%   v, acc, phy, w  - 稠密轨迹速度、加速度、转角、转角速度
%   terminal_time   - 稠密轨迹总时间
%
% 输出：
%   x1, y1, theta1  - 筛选后的 NLP 配置点位姿
%   v, acc, phy, w  - 筛选后的节点变量
%   time            - 每个 NLP 区间的时间长度，末尾补 0
%
% 核心思想：
%   从起点开始贪心向后搜索，在 [min_dt, max_dt] 允许范围内尽量选远点。
%   若某个候选区间 EF 不可行，则回退到最后一个可行区间，并乘以
%   config_shrink_scale 做保守收缩。

global params

%% ================================================================
% Module 1: 输入轨迹标准化
% ================================================================
% 将所有输入轨迹变量统一转换为行向量，避免后续索引时因行列方向不同出错。
% dense_n 表示稠密轨迹点数。
% base_dt 是稠密轨迹相邻采样点之间的基础时间间隔。

x1     = x1(:).';
y1     = y1(:).';
theta1 = theta1(:).';
v      = v(:).';
acc    = acc(:).';
phy    = phy(:).';
w      = w(:).';

dense_n = length(x1);
base_dt = terminal_time / (dense_n - 1);


%% ================================================================
% Module 2: 读取 NLP1 中的时间上下界
% ================================================================
% NLP1.mod 中每个离散区间 dt 需要满足：
%       min_dt <= dt(i) <= max_dt
%
% 这里将连续时间上下界转换为稠密采样点的索引跨度上下界：
%       min_step <= selected(i+1)-selected(i) <= max_step

min_dt = params.ef.min_dt;
max_dt = params.ef.max_dt;

if min_dt <= 0 || max_dt <= 0 || min_dt > max_dt
    error('Invalid NLP time bounds: params.ef.min_dt=%.6g, params.ef.max_dt=%.6g.', ...
        min_dt, max_dt);
end

min_step = max(1, ceil(min_dt / base_dt));
max_step = max(min_step, floor(max_dt / base_dt));


%% ================================================================
% Module 3: 读取 EF 配置点收缩比例
% ================================================================
% shrink_scale 只作用于"选点索引间隔"，不缩小 EF box 本身。
%
% 例如：
%   step = 1,2,3,4 可行，step = 5 不可行，
%   若 shrink_scale = 0.9，则最终选择 floor(4*0.9)=3。
%
% 这样做的目的是避免选点刚好贴着 EF 可行边界，提高初始解保守性。

shrink_scale = getConfigShrinkScale();


%% ================================================================
% Module 4: 初始化筛选变量
% ================================================================
% selected      : 被选中的稠密轨迹索引，起点必须选中
% interval_time : 相邻两个被选中点之间的时间间隔
% abox~dbox     : 每个 EF 区间对应的外扩尺度
% cur_idx       : 当前已选中的配置点索引

selected = 1;
interval_time = [];
abox = [];
bbox = [];
cbox = [];
dbox = [];
cur_idx = 1;

% ================================================================
% 强制保留换向点
% ================================================================
if isfield(params.ef, 'dense_change_idx') &&~isempty(params.ef.dense_change_idx)
    mandatory_idx = params.ef.dense_change_idx(:).';
    mandatory_idx = mandatory_idx(mandatory_idx > 1 & mandatory_idx < dense_n);
    mandatory_idx = unique(mandatory_idx, 'stable');
else
    mandatory_idx = [];
end


%% ================================================================
% Module 5: 基于 EF 可行性的贪心降采样
% ================================================================
% 每次从当前点 cur_idx 出发，向后寻找下一个 NLP 配置点。
%
% 候选区间必须满足：
%   1) 时间长度满足 NLP1 的 dt 上下界；
%   2) EF 扩展包络盒不碰撞障碍物；
%   3) EF 几何前提约束成立；
%   4) 尽量选择更远的点，以减少 NLP 离散点数量。
%
% 如果遇到第一个不可行 step，则回退到最后一个可行 step，
% 并乘 shrink_scale 做保守收缩。

while cur_idx < dense_n

    remaining_step = dense_n - cur_idx;

    % ============================================================
    % 当前区间绝不能越过下一个 mandatory point
    % ============================================================
    next_mandatory = mandatory_idx(find(mandatory_idx > cur_idx, 1, 'first'));

    if isempty(next_mandatory)
        next_mandatory = dense_n;
    end

    step_to_mandatory = next_mandatory - cur_idx;

    % 如果两个强制点本身的时间距离小于 min_dt，
    % 则无法同时满足"保留 cusp"和 NLP dt 下界。
    if step_to_mandatory < min_step
        error(['TimeDistribution: mandatory point is too close. ', 'cur_idx=%d, mandatory_idx=%d, step=%d, min_step=%d.'], cur_idx, next_mandatory, step_to_mandatory, min_step);
    end

    % ------------------------------------------------------------
    % Case 1: 当前点到终点的剩余跨度不超过 max_step
    % ------------------------------------------------------------
    % 若终点已经落在允许的最大时间间隔内，则直接连接到终点。
    % 注意：这里默认终点区间可接受，只估计 EF box，不再做逐步搜索。
    if next_mandatory == dense_n && remaining_step <= max_step

        next_idx = dense_n;

        [a_temp, b_temp, c_temp, d_temp] = ...
            estimateIntervalBox(cur_idx, next_idx, x1, y1, v, phy);

        selected(end+1)      = next_idx;
        interval_time(end+1) = (next_idx - cur_idx) * base_dt;

        abox(end+1) = a_temp;
        bbox(end+1) = b_temp;
        cbox(end+1) = c_temp;
        dbox(end+1) = d_temp;

        break;
    end


    % ------------------------------------------------------------
    % Case 2: 正常区间搜索
    % ------------------------------------------------------------
    % upper_step 初始取 max_step。
    % 若按照 max_step 选点会导致最后剩余区间小于 min_step，
    % 则提前缩短本次 upper_step，给最后一个区间留出至少 min_step。
    upper_step = min(max_step, step_to_mandatory);
    remaining_to_boundary = step_to_mandatory - upper_step;

    if remaining_to_boundary > 0 && remaining_to_boundary < min_step
        upper_step = step_to_mandatory - min_step;
    end

    upper_step = max(min_step, upper_step);


    % ------------------------------------------------------------
    % 逐步测试候选区间
    % ------------------------------------------------------------
    % last_valid_step 记录最后一个可行 step。
    % first_invalid_step 记录第一个不可行 step。
    % last_valid_box 记录最后一个可行区间对应的 EF box 尺寸。
    last_valid_step = [];
    first_invalid_step = [];
    last_valid_box = [];

    for step = min_step:upper_step

        cand_idx = cur_idx + step;

        [is_valid, box_dim] = ...
            isIntervalValid(cur_idx, cand_idx, base_dt, x1, y1, theta1, v, phy);

        if is_valid
            last_valid_step = step;
            last_valid_box = box_dim;
        else
            first_invalid_step = step;
            break;
        end
    end


    % ------------------------------------------------------------
    % 根据可行性搜索结果确定最终 chosen_step
    % ------------------------------------------------------------
    if isempty(first_invalid_step)

        % 所有候选 step 均可行，则选最大允许步长 upper_step。
        chosen_step = upper_step;

        if isempty(last_valid_box)
            [~, last_valid_box] = ...
                isIntervalValid(cur_idx, cur_idx + chosen_step, base_dt, x1, y1, theta1, v, phy);
        end

    else

        % 存在不可行 step。
        if isempty(last_valid_step)

            % 连 min_step 都不可行。
            % 为避免死循环，只能强制推进 min_step。
            % 这通常意味着当前初始轨迹在该处 EF 已经不可行。
            chosen_step = min_step;

        else

            % 回退到最后一个可行 step，并乘 shrink_scale 做保守收缩。
            chosen_step = max(min_step, floor(last_valid_step * shrink_scale));
        end

        chosen_step = min(chosen_step, upper_step);

        [~, last_valid_box] = ...
            isIntervalValid(cur_idx, cur_idx + chosen_step, base_dt, x1, y1, theta1, v, phy);
    end


    % ------------------------------------------------------------
    % 终点剩余区间保护
    % ------------------------------------------------------------
    % 如果选择 next_idx 后，剩余到终点的时间小于 min_dt，
    % 则直接把终点作为下一个点，避免最后产生非法短区间。
    next_idx = cur_idx + chosen_step;
    remaining_to_boundary = next_mandatory - next_idx;
    if remaining_to_boundary > 0 && remaining_to_boundary < min_step
        next_idx = next_mandatory;
        chosen_step = next_idx - cur_idx;
        [~, last_valid_box] = isIntervalValid(cur_idx, next_idx, base_dt, x1, y1, theta1, v, phy);
    end


    % ------------------------------------------------------------
    % 保存本次选点结果
    % ------------------------------------------------------------
    selected(end+1)      = next_idx;                 %#ok<AGROW>
    interval_time(end+1) = chosen_step * base_dt;    %#ok<AGROW>

    abox(end+1) = last_valid_box(1);                 %#ok<AGROW>
    bbox(end+1) = last_valid_box(2);                 %#ok<AGROW>
    cbox(end+1) = last_valid_box(3);                 %#ok<AGROW>
    dbox(end+1) = last_valid_box(4);                 %#ok<AGROW>

    cur_idx = next_idx;
end


%% ================================================================
% Module 6: 根据 selected 执行轨迹降采样
% ================================================================
% selected 中保存的是原稠密轨迹索引。
% params.nfe 是最终 NLP1 的离散节点数。

selected = unique(selected, 'stable');
params.nfe = length(selected);

x1     = x1(selected);
y1     = y1(selected);
theta1 = theta1(selected);
v      = v(selected);
acc    = acc(selected);
phy    = phy(selected);
w      = w(selected);

% time 的长度通常与 NLP 节点数一致：
% 前 Nfe-1 个元素表示区间 dt，最后一个补 0。
time = [interval_time, 0];

validateTimeBounds(interval_time, min_dt, max_dt);


%% ================================================================
% Module 7: 缓存 EF 初始轨迹信息
% ================================================================
% 将筛选后的轨迹和 EF box 尺寸存入 params.ef，
% 方便 WriteEFInitialGuess、可视化、初始解检查函数调用。

params.ef.x     = x1(:);
params.ef.y     = y1(:);
params.ef.theta = theta1(:);
params.ef.v     = v(:);
params.ef.acc   = acc(:);
params.ef.phy   = phy(:);
params.ef.w     = w(:);
params.ef.time  = time(:);

params.ef.a_box = abox(:);
params.ef.b_box = bbox(:);
params.ef.c_box = cbox(:);
params.ef.d_box = dbox(:);

params.ef.config_shrink_scale = shrink_scale;


%% ================================================================
% Module 8: 检测换向点
% ================================================================
% 根据筛选后的速度符号变化检测换向节点。
% change_final 可用于后续特殊处理换向点附近的约束或初始值。

params.ef.change_final = detectDirectionSwitch(v);


%% ================================================================
% Module 9: 输出调试信息
% ================================================================

fprintf('EF selected point count: %d\n', params.nfe);
fprintf('EF interval dt range   : %.6e / %.6e\n', min(interval_time), max(interval_time));
fprintf('EF config shrink scale : %.6f\n', shrink_scale);
fprintf('EF direction switches  : %d\n', numel(params.ef.change_final));

end


%% ========================================================================
% Local Function 1: 读取配置点收缩比例
% ========================================================================
function shrink_scale = getConfigShrinkScale()
%GETCONFIGSHRINKSCALE  获取 EF 配置点收缩比例
%
% 优先级：
%   1) params.ef.config_shrink_scale
%   2) params.scheme1.config_shrink_scale
%   3) 默认值 0.9
%
% 输出：
%   shrink_scale ∈ [0, 1]

global params

shrink_scale = 0.9;

if isfield(params, 'ef') && isfield(params.ef, 'config_shrink_scale')
    shrink_scale = params.ef.config_shrink_scale;
elseif isfield(params, 'scheme1') && isfield(params.scheme1, 'config_shrink_scale')
    shrink_scale = params.scheme1.config_shrink_scale;
end

shrink_scale = min(max(shrink_scale, 0), 1);

end


%% ========================================================================
% Local Function 2: 判断候选 EF 区间是否可行
% ========================================================================
function [is_valid, box_dim] = isIntervalValid(cur_idx, cand_idx, base_dt, x, y, theta, v, phy)
%ISINTERVALVALID  检查从 cur_idx 到 cand_idx 的 EF 区间是否合法
%
% 检查内容：
%   1) 根据当前区间长度估计 EF box 外扩尺度；
%   2) 计算 EF box 的四个顶点；
%   3) 检查 EF box 是否与障碍物冲突；
%   4) 检查 EF 理论成立的几何前提约束。
%
% 输出：
%   is_valid : true 表示该区间可作为 NLP 相邻配置点区间
%   box_dim  : [a_temp, b_temp, c_temp, d_temp]

global params

delta_t = (cand_idx - cur_idx) * base_dt;
motion_idx = getIntervalMotionIndex(v, cur_idx, cand_idx);
interval_v = v(cur_idx:cand_idx);
has_forward = any(interval_v >  1e-6);
has_reverse = any(interval_v < -1e-6);

if has_forward && has_reverse
    is_valid = false;
    box_dim = [0, 0, 0, 0];
    return;
end

[a_temp, b_temp, c_temp, d_temp] = estimateIntervalBox(cur_idx, cand_idx,x, y, v, phy);
box_dim = [a_temp, b_temp, c_temp, d_temp];

% 当前 interval 的代表曲率
kappa_k = tan(phy(motion_idx)) / params.vehicle.lw;
abs_kappa_k = abs(kappa_k);

% 当前 interval 的实际几何弧长
dx_interval = diff(x(cur_idx:cand_idx));
dy_interval = diff(y(cur_idx:cand_idx));

s_abs = sum(hypot(dx_interval, dy_interval));

% 运动方向由当前 interval 第一个非零运动点决定
sgn = sign(v(motion_idx));

if sgn == 0
    signed_s = 0;
else
    signed_s = sgn * s_abs;
end

% 前进距离和倒车距离分解
splus  = smoothPlus(signed_s);
sminus = smoothPlus(-signed_s);

% 计算 EF box 四个顶点
[AX, AY, BX, BY, CX, CY, DX, DY] = getEFBoxVertices(x(cur_idx), y(cur_idx), theta(cur_idx),a_temp, b_temp, c_temp, d_temp);


% ------------------------------------------------------------
% EF 前提约束
% ------------------------------------------------------------
% 这些条件与 NLP1.mod 中 EF 相关约束对应。
% 此处不使用 threshold_rate 进行额外收紧，只判断原始前提是否失效。

cond_arc = abs_kappa_k * (splus + sminus) > 1.5708;

cond_f1 = (1 + params.vehicle.hlb * abs_kappa_k) * tan(abs_kappa_k * splus) > ...
    params.vehicle.lr * abs_kappa_k;

cond_f2 = abs_kappa_k * params.vehicle.LF * tan(abs_kappa_k * splus) > ...
    1 + params.vehicle.hlb * abs_kappa_k;

cond_r1 = (1 + params.vehicle.hlb * abs_kappa_k) * tan(abs_kappa_k * sminus) > ...
    params.vehicle.LF * abs_kappa_k;

cond_r2 = abs_kappa_k * params.vehicle.lr * tan(abs_kappa_k * sminus) > ...
    1 + params.vehicle.hlb * abs_kappa_k;


% ------------------------------------------------------------
% 综合可行性判断
% ------------------------------------------------------------
% 只要 EF box 撞障碍物，或者任意 EF 前提条件失效，
% 当前候选区间即判定为不可行。
is_valid = ~(IsEnlargedBoxInvalid(AX, AY, BX, BY, CX, CY, DX, DY) || ...
    cond_arc || cond_f1 || cond_f2 || cond_r1 || cond_r2);

end


%% ========================================================================
% Local Function 3: 计算 EF box 四个顶点
% ========================================================================
function [AX, AY, BX, BY, CX, CY, DX, DY] = getEFBoxVertices(x, y, theta, a, b, c, d)
%GETEFBOXVERTICES  根据车辆位姿和 EF 外扩尺度计算扩展矩形顶点
%
% EF box 尺寸含义：
%   a : 前向外扩距离
%   c : 后向外扩距离
%   b : 左侧外扩距离
%   d : 右侧外扩距离
%
% 顶点顺序：
%   A : 前左角
%   B : 前右角
%   C : 后右角
%   D : 后左角

global params

LF  = params.vehicle.LF;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

ct = cos(theta);
st = sin(theta);

AX = x + (LF + a) * ct - (hlb + b) * st;
AY = y + (LF + a) * st + (hlb + b) * ct;

BX = x + (LF + a) * ct + (hlb + d) * st;
BY = y + (LF + a) * st - (hlb + d) * ct;

CX = x - (lr + c) * ct + (hlb + d) * st;
CY = y - (lr + c) * st - (hlb + d) * ct;

DX = x - (lr + c) * ct - (hlb + b) * st;
DY = y - (lr + c) * st + (hlb + b) * ct;

end


%% ========================================================================
% Local Function 4: 估计候选区间 EF 外扩尺度
% ========================================================================
function [a_temp, b_temp, c_temp, d_temp] = estimateIntervalBox(cur_idx, cand_idx, x, y, v, phy)

global params

% 当前 interval 第一个真正运动的 dense 点
motion_idx = getIntervalMotionIndex(v, cur_idx, cand_idx);

% dense trajectory 在这个 interval 中的实际几何行驶距离
dx = diff(x(cur_idx:cand_idx));
dy = diff(y(cur_idx:cand_idx));

s_k = sum(hypot(dx, dy));

% 使用当前实际运动段的代表曲率
kappa_k = tan(phy(motion_idx)) / params.vehicle.lw;

% 使用当前 interval 的实际方向
sgn = sign(v(motion_idx));

% 极端退化情况
if sgn == 0
    sgn = 1;
end

[a_temp, b_temp, c_temp, d_temp] = ...
    EstimateAABBnew(s_k, kappa_k, sgn);

end

%% ========================================================================
% Local Function 6: 校验最终时间间隔是否合法
% ========================================================================
function validateTimeBounds(interval_time, min_dt, max_dt)
%VALIDATETIMEBOUNDS  检查最终生成的 interval_time 是否满足 NLP1 时间约束
%
% 若任意区间时间不在 [min_dt, max_dt] 内，则直接报错。
% 这一步用于防止贪心选点后产生非法 dt。

tol = 1e-10;

if any(interval_time < min_dt - tol) || any(interval_time > max_dt + tol)
    error('TimeDistribution produced dt outside NLP1 bounds: min/max dt = %.12g / %.12g, bounds = %.12g / %.12g.', ...
        min(interval_time), max(interval_time), min_dt, max_dt);
end

end


%% ========================================================================
% Local Function 7: 检测换向点
% ========================================================================
function chg_ef = detectDirectionSwitch(v)
%DETECTDIRECTIONSWITCH  检测筛选后轨迹中的速度换向点
%
% 判断规则：
%   v >  thr  视为前进
%   v < -thr  视为倒车
%   |v| <= thr 视为零速点
%
% 若两个非零速度点之间符号发生变化，则记录当前索引为换向点。
% 零速点会被跳过，不直接作为换向判断依据。

thr = 0.05;

v_sign = zeros(size(v));
v_sign(v >  thr) = 1;
v_sign(v < -thr) = -1;

chg_ef = [];

for i = 2:length(v_sign)

    % 当前点为零速点，不参与换向判断
    if v_sign(i) == 0
        continue;
    end

    % 向前寻找最近一个非零速度符号
    j = i - 1;

    while j >= 1 && v_sign(j) == 0
        j = j - 1;
    end

    % 当前非零速度符号与前一个非零速度符号不同，则发生换向
    if j >= 1 && v_sign(i) ~= v_sign(j)
        chg_ef(end+1) = i; %#ok<AGROW>
    end
end

end

function val = smoothPlus(x)
global params
alpha = 60;
if isfield(params, 'nlp') && isfield(params.nlp, 'alpha') && ~isempty(params.nlp.alpha)
    alpha = params.nlp.alpha;
end
m = max(0, x);
val = m + log(exp(alpha * (0 - m)) + exp(alpha * (x - m))) / alpha;
end

function motion_idx = getIntervalMotionIndex(v, cur_idx, cand_idx)
thr = 1e-6;
motion_idx = [];
for jj = cur_idx:cand_idx
    if abs(v(jj)) >= thr
        motion_idx = jj;
        break;
    end
end
if isempty(motion_idx)
    motion_idx = cur_idx;
end
end

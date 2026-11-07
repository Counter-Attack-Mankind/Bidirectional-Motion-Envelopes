function result = EvaluateOptimizationResult(scheme_dir, task_id)

global params

%% ============================================================
% 1. 基本路径
% ============================================================

task_id = round(task_id);
runtime_dir = fullfile(scheme_dir, 'runtime');
result_root = fullfile(scheme_dir,'Results',sprintf('task_%02d', task_id));
log_file = fullfile(result_root, 'ampl_log.txt');
experiment_root = fileparts(scheme_dir);
output_csv = fullfile(experiment_root, 'results.csv');

%% ============================================================
% 2. 初始化结果
% ============================================================

result = struct();
result.task_id = task_id;
result.success = 0;
result.inf_pr_0 = NaN;
result.ipopt_iterations = NaN;
result.ipopt_cpu_sec = NaN;
result.total_frames = NaN;
result.collision_frames = NaN;
result.collision_percent = NaN;

%% ============================================================
% 3. 读取优化成功标志
% ============================================================

flag_file = fullfile(runtime_dir, 'opti_flag.txt');
if exist(flag_file, 'file') == 2
    opti_flag = load(flag_file);
    if ~isempty(opti_flag)
        result.success = double(opti_flag(1) ~= 0);
    end
else
    warning('EvaluateOptimizationResult:MissingFlag','opti_flag.txt does not exist: %s', flag_file);
    result.success = 0;
end


%% ============================================================
% 4. 若优化成功，计算 CPU 时间和碰撞百分比
% ============================================================

result.inf_pr_0 = parseInitialPrimalInfeasibility(log_file);
result.ipopt_iterations = parseIpoptIterationCount(log_file);
result.ipopt_cpu_sec = parseIpoptCpuTime(log_file);

if result.success == 1

    x = readRequiredVector(runtime_dir, 'x.txt');
    y = readRequiredVector(runtime_dir, 'y.txt');
    theta = readRequiredVector(runtime_dir, 'theta.txt');
    dt = readRequiredVector(runtime_dir, 'dt.txt');
    x = x(:);
    y = y(:);
    theta = theta(:);
    dt = dt(:);
    Nfe = numel(x);
    assert(numel(y) == Nfe, 'Optimized y length does not match x.');
    assert(numel(theta) == Nfe, 'Optimized theta length does not match x.');
    assert(numel(dt) == Nfe - 1, 'Optimized dt length must be Nfe-1.');

    % --------------------------------------------------------
    % 对 theta 连续化
    % --------------------------------------------------------

    theta = unwrapTheta(theta);

    % --------------------------------------------------------
    % 稠密时间插值
    %
    % 固定 100 Hz：
    %     eval_dt = 0.01 s
    %
    % 对不同 Scheme 使用完全相同的评价频率。
    % --------------------------------------------------------

    eval_dt = 0.01;
    [x_dense, y_dense, theta_dense] = interpolateOptimizedTrajectory(x, y, theta, dt, eval_dt);

    % --------------------------------------------------------
    % SAT 碰撞检测
    % --------------------------------------------------------

    [collision_mask, collision_frames] = evaluateDenseTrajectoryCollision(x_dense, y_dense,theta_dense);
    total_frames = numel(collision_mask);

    % --------------------------------------------------------
    % 4.6 碰撞百分比
    % --------------------------------------------------------

    if total_frames > 0
        collision_percent = collision_frames / total_frames * 100;
    else
        collision_percent = NaN;
    end
    result.total_frames = total_frames;
    result.collision_frames = collision_frames;
    result.collision_percent = collision_percent;

end


%% ============================================================
% 5. 打印结果
% ============================================================

fprintf('\n');
fprintf('========== Optimization Evaluation ==========\n');
fprintf('task_id               : %d\n', result.task_id);
fprintf('success               : %d\n', result.success);
fprintf('initial inf_pr         : %.3f\n', result.inf_pr_0);
fprintf('IPOPT iterations       : %.0f\n', result.ipopt_iterations);

if result.success == 1

    fprintf('IPOPT CPU time         : %.6f s\n', ...
        result.ipopt_cpu_sec);

    fprintf('dense frame count      : %d\n', ...
        result.total_frames);

    fprintf('collision frame count  : %d\n', ...
        result.collision_frames);

    fprintf('collision percent      : %.6f %%\n', ...
        result.collision_percent);

else

    fprintf('IPOPT CPU time         : %.6f s\n',result.ipopt_cpu_sec);
    fprintf('collision percent      : NaN\n');

end

fprintf('=============================================\n\n');


%% ============================================================
% 6. Update the root-level unified results.csv.
% ============================================================

UpdateUnifiedResults(output_csv, scheme_dir, result);

end


%% ========================================================================
% Local Function 1
% Parse the iteration-0 unscaled original-constraint violation.
% IPOPT may first push supplied initial values into variable bounds.
% ========================================================================
function inf_pr_0 = parseInitialPrimalInfeasibility(log_file)

inf_pr_0 = NaN;

if exist(log_file, 'file') ~= 2
    warning( ...
        'EvaluateOptimizationResult:MissingLog', ...
        'AMPL log file does not exist: %s', ...
        log_file);
    return;
end

log_text = fileread(log_file);
tokens = regexp( ...
    log_text, ...
    '(?m)^\s*0\s+[0-9Ee+\-.]+\s+([0-9Ee+\-.]+)\s+', ...
    'tokens');

if numel(tokens) ~= 1
    warning( ...
        'EvaluateOptimizationResult:InitialPrimalInfeasibilityNotFound', ...
        'Expected one IPOPT iteration-0 row in %s; found %d.', ...
        log_file, numel(tokens));
    return;
end

inf_pr_0 = str2double(tokens{1}{1});

end


%% ========================================================================
% Local Function 2
% Parse IPOPT's authoritative final iteration count, including restoration
% iterations. A time-limited count is a stopped, not converged, observation.
% ========================================================================
function iterations = parseIpoptIterationCount(log_file)

iterations = NaN;

if exist(log_file, 'file') ~= 2
    warning( ...
        'EvaluateOptimizationResult:MissingLog', ...
        'AMPL log file does not exist: %s', ...
        log_file);
    return;
end

log_text = fileread(log_file);
tokens = regexp( ...
    log_text, ...
    'Number of Iterations\.\.\.\.:\s*(\d+)', ...
    'tokens');

if numel(tokens) ~= 1
    warning( ...
        'EvaluateOptimizationResult:IpoptIterationCountNotFound', ...
        'Expected one IPOPT iteration summary in %s; found %d.', ...
        log_file, numel(tokens));
    return;
end


iterations = str2double(tokens{1}{1});

end


%% ========================================================================
% Local Function 3
% Total solver CPU =
%   IPOPT internal CPU (w/o function evaluations)
% + NLP function evaluation CPU
% ========================================================================
function cpu_sec = parseIpoptCpuTime(log_file)

cpu_sec = NaN;

if exist(log_file, 'file') ~= 2
    warning( ...
        'EvaluateOptimizationResult:MissingLog', ...
        'AMPL log file does not exist: %s', ...
        log_file);
    return;
end

log_text = fileread(log_file);

% ------------------------------------------------------------
% 1. IPOPT 内部 CPU 时间，不包含 NLP 函数求值
% ------------------------------------------------------------
pattern_ipopt = ...
    'Total CPU secs in IPOPT \(w/o function evaluations\)\s*=\s*([0-9Ee+\-.]+)';

token_ipopt = regexp( ...
    log_text, ...
    pattern_ipopt, ...
    'tokens', ...
    'once');

% ------------------------------------------------------------
% 2. NLP 函数求值 CPU 时间
% ------------------------------------------------------------
pattern_eval = ...
    'Total CPU secs in NLP function evaluations\s*=\s*([0-9Ee+\-.]+)';

token_eval = regexp( ...
    log_text, ...
    pattern_eval, ...
    'tokens', ...
    'once');

% ------------------------------------------------------------
% 3. 检查日志是否完整
% ------------------------------------------------------------
if isempty(token_ipopt)
    warning( ...
        'EvaluateOptimizationResult:IpoptCpuTimeNotFound', ...
        ['Cannot find "Total CPU secs in IPOPT ', ...
         '(w/o function evaluations)" in %s.'], ...
        log_file);
    return;
end

if isempty(token_eval)
    warning( ...
        'EvaluateOptimizationResult:FunctionEvalCpuTimeNotFound', ...
        ['Cannot find "Total CPU secs in NLP function evaluations" ', ...
         'in %s.'], ...
        log_file);
    return;
end

% ------------------------------------------------------------
% 4. 总求解 CPU 时间
% ------------------------------------------------------------
cpu_ipopt = str2double(token_ipopt{1});
cpu_eval  = str2double(token_eval{1});

cpu_sec = cpu_ipopt + cpu_eval;

end


%% ========================================================================
% Local Function 4
% 读取必要的优化变量
% ========================================================================
function vec = readRequiredVector(runtime_dir, filename)

file = fullfile(runtime_dir, filename);

if exist(file, 'file') ~= 2
    error( ...
        'EvaluateOptimizationResult:MissingResult', ...
        'Missing optimized result file: %s', ...
        file);
end

vec = load(file);
vec = vec(:);

end


%% ========================================================================
% Local Function 5
% theta 连续化
% ========================================================================
function theta = unwrapTheta(theta)

theta = theta(:);

for i = 2:numel(theta)

    while theta(i) - theta(i-1) > pi
        theta(i) = theta(i) - 2*pi;
    end

    while theta(i) - theta(i-1) < -pi
        theta(i) = theta(i) + 2*pi;
    end

end

end


%% ========================================================================
% Local Function 6
% 对优化后的 Nfe 轨迹进行固定时间步长线性插值
% ========================================================================
function [x_dense, y_dense, theta_dense] = interpolateOptimizedTrajectory(x, y, theta, dt, eval_dt)

x_dense = [];
y_dense = [];
theta_dense = [];
Nseg = numel(dt);

for i = 1:Nseg

    if dt(i) <= 0
        error('EvaluateOptimizationResult:InvalidDt', 'dt(%d) = %.12g is not positive.', i, dt(i));
    end

    % 该 interval 至少分成 1 段
    n_interval = max(1, ceil(dt(i) / eval_dt));

    % 包含两个端点
    tau = linspace(0, 1, n_interval + 1);
    x_seg = x(i) + tau * (x(i+1) - x(i));
    y_seg = y(i) + tau * (y(i+1) - y(i));
    theta_seg = theta(i) + tau * (theta(i+1) - theta(i));

    % 避免相邻 interval 的公共端点重复计数，也就是对[A,B)做
    if i > 1
        x_seg(1) = [];
        y_seg(1) = [];
        theta_seg(1) = [];
    end

    x_dense = [x_dense, x_seg]; %#ok<AGROW>
    y_dense = [y_dense, y_seg]; %#ok<AGROW>
    theta_dense = [theta_dense, theta_seg]; %#ok<AGROW>

end

x_dense = x_dense(:);
y_dense = y_dense(:);
theta_dense = theta_dense(:);

end


%% ========================================================================
% Local Function 7
% 对全部稠密轨迹帧执行真实车身 SAT 碰撞检测
% ========================================================================
function [collision_mask, collision_frames] = ...
    evaluateDenseTrajectoryCollision( ...
        x, y, theta)

global params

n = numel(x);

collision_mask = false(n, 1);

for k = 1:n

    vehicle_poly = buildVehiclePolygon( ...
        x(k), ...
        y(k), ...
        theta(k));

    frame_collision = false;

    for obs_idx = 1:params.environment.num_obs

        obs_x = params.environment.obs(obs_idx).x(:);
        obs_y = params.environment.obs(obs_idx).y(:);

        % 删除首尾重复闭合点
        if numel(obs_x) >= 2 && ...
                abs(obs_x(1) - obs_x(end)) < 1e-12 && ...
                abs(obs_y(1) - obs_y(end)) < 1e-12

            obs_x(end) = [];
            obs_y(end) = [];
        end

        obstacle_poly = [obs_x, obs_y];

        if polygonsIntersectSAT(vehicle_poly, obstacle_poly)
            frame_collision = true;
            break;
        end

    end

    collision_mask(k) = frame_collision;

end

collision_frames = nnz(collision_mask);

end


%% ========================================================================
% Local Function 8
% 构造真实车辆矩形
%
% 与 PlotTrueVehicleSweptAreaOnly 中真实车体定义一致。
% ========================================================================
function vehicle_poly = buildVehiclePolygon(x, y, theta)

global params

lf  = params.vehicle.lf;
lw  = params.vehicle.lw;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

c = cos(theta);
s = sin(theta);

AX = x + (lf + lw) * c - hlb * s;
AY = y + (lf + lw) * s + hlb * c;

BX = x + (lf + lw) * c + hlb * s;
BY = y + (lf + lw) * s - hlb * c;

CX = x - lr * c + hlb * s;
CY = y - lr * s - hlb * c;

DX = x - lr * c - hlb * s;
DY = y - lr * s + hlb * c;

vehicle_poly = [ ...
    AX, AY;
    BX, BY;
    CX, CY;
    DX, DY];

end


%% ========================================================================
% Local Function 9
% SAT 多边形碰撞检测
%
% 适用于当前实验中的凸多边形：
%   - 车辆矩形
%   - 三角形 / 四边形障碍物
%
% 若任意一个分离轴上投影不重叠，则无碰撞。
% 所有轴均重叠，则判定碰撞。
%
% 接触也按碰撞处理。
% ========================================================================
function is_collision = polygonsIntersectSAT(poly1, poly2)

tol = 1e-10;

axes1 = polygonAxes(poly1);
axes2 = polygonAxes(poly2);

axes_all = [axes1; axes2];

for i = 1:size(axes_all, 1)

    axis_vec = axes_all(i, :);

    proj1 = poly1 * axis_vec.';
    proj2 = poly2 * axis_vec.';

    min1 = min(proj1);
    max1 = max(proj1);

    min2 = min(proj2);
    max2 = max(proj2);

    % 存在严格分离轴 -> 不碰撞
    if max1 < min2 - tol || ...
       max2 < min1 - tol

        is_collision = false;
        return;
    end

end

% 没找到分离轴
is_collision = true;

end


%% ========================================================================
% Local Function 10
% 获取 SAT 所需的边法向量
% ========================================================================
function axes_out = polygonAxes(poly)

n = size(poly, 1);

axes_out = zeros(n, 2);

valid_count = 0;

for i = 1:n

    j = i + 1;

    if j > n
        j = 1;
    end

    edge = poly(j, :) - poly(i, :);

    axis_vec = [-edge(2), edge(1)];

    norm_axis = norm(axis_vec);

    % 重复点产生零长度边时忽略
    if norm_axis < 1e-12
        continue;
    end

    axis_vec = axis_vec / norm_axis;

    valid_count = valid_count + 1;
    axes_out(valid_count, :) = axis_vec;

end

axes_out = axes_out(1:valid_count, :);

end



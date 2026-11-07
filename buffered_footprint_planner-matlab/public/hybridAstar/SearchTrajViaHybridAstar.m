function success = SearchTrajViaHybridAstar()

success = 0;
global params openlist_ grid_space_ ha_debug_

%% =========================
% 1. Guiding path
% =========================
SearchGuidingPath();
EstimateprogressAlongGuidingLine();

ha_debug_.last_invalid = {};
ha_debug_.max_invalid_keep = 5;
ha_debug_.last_cur_node = [];
ha_debug_.last_cur_path_x = [];
ha_debug_.last_cur_path_y = [];

if params.ha.enable_debug_plot
    InitHADebugFigure();
end

%% =========================
% 2. Initialize Hybrid A*
% =========================
grid_space_ = cell(params.ha.nx, params.ha.ny, params.ha.ntheta);
openlist_ = [];

init_node.x = params.task.x0;
init_node.y = params.task.y0;
init_node.theta = params.task.theta0;
init_node.v = 0;
init_node.phy = 0;
init_node.g = 0;
init_node.h = CalculateH(init_node);
init_node.f = init_node.g + init_node.h;
init_node.parent_id = [-1 -1 -1];
init_node.from_parent = [];
init_node.local_dt = 0;
init_node.is_closed = 0;
init_node.is_open = 1;

init_id = CalculateNodeIndex(init_node);
grid_space_{init_id(1), init_id(2), init_id(3)} = init_node;
openlist_ = [init_node.f, init_id];

expansion_pattern = SpecifySamplePattern();

completeness_flag = 0;
complete_via_rs = false;
rs_seg = [];
rs_counter = 0;

cur_best_node = init_node;
cur_best_node_cost_val = inf;

iter = 0;
search_tic = tic;
fail_reason = '';

%% =========================
% 3. Hybrid A* search
% =========================
while ~isempty(openlist_)

    iter = iter + 1;

    if iter > params.ha.max_iter
        fail_reason = '达到最大迭代次数';
        break;
    end

    if toc(search_tic) > params.ha.max_search_time
        fail_reason = '搜索时间超过上限';
        break;
    end

    [cur_node, cur_node_id] = ExtractMinFNodeFromOpenlist();

    if params.ha.enable_debug_plot && mod(iter, params.ha.debug_plot_stride) == 0
        ha_debug_.last_cur_node = cur_node;
        [ha_debug_.last_cur_path_x, ha_debug_.last_cur_path_y] = TraceNodePath(cur_node);
        UpdateHADebugFigureSimple(iter, cur_node, cur_best_node);
        drawnow limitrate;
    end

    parent_phy = cur_node.phy;

    for ii = 1:size(expansion_pattern, 1)

        child_node.v = expansion_pattern(ii, 1);
        child_node.phy = expansion_pattern(ii, 2);

        local_dt = params.ha.simu_unit_duration;
        path_seg = SimulateForward_test(cur_node, child_node.v, child_node.phy, local_dt);

        child_node.x = path_seg.x(end);
        child_node.y = path_seg.y(end);
        child_node.theta = path_seg.theta(end);
        child_node.parent_id = cur_node_id;
        child_node.from_parent = path_seg;
        child_node.local_dt = local_dt;

        child_node_id = CalculateNodeIndex(child_node);

        % 已关闭节点直接跳过
        if ~isempty(grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)})
            if grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)}.is_closed
                continue;
            end
        end

        % Hybrid A* 只检查真实车身
        if ~IsPathValid(path_seg)

            if params.ha.enable_debug_plot
                dbg.x = child_node.x;
                dbg.y = child_node.y;
                dbg.theta = child_node.theta;
                dbg.phy = child_node.phy;
                dbg.dir = child_node.v;
                dbg.path_seg = path_seg;

                ha_debug_.last_invalid{end+1} = dbg;

                if numel(ha_debug_.last_invalid) > ha_debug_.max_invalid_keep
                    ha_debug_.last_invalid = ha_debug_.last_invalid(end-ha_debug_.max_invalid_keep+1:end);
                end
            end

            continue;
        end

        %% -------------------------
        % Cost
        % --------------------------
        penalty_phy = abs(child_node.phy - parent_phy) * params.ha.penalty_on_phy_change;

        if child_node.v < 0
            penalty_backward = local_dt * params.ha.penalty_for_backward;
        else
            penalty_backward = 0;
        end

        child_candidate_g = cur_node.g + local_dt + penalty_phy + penalty_backward;

        %% -------------------------
        % Insert / update node
        % --------------------------
        node_accepted = false;

        if isempty(grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)})

            child_node.g = child_candidate_g;
            child_node.h = CalculateH(child_node);
            child_node.f = child_node.g + child_node.h;
            child_node.is_closed = 0;
            child_node.is_open = 1;

            grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)} = child_node;
            openlist_ = [openlist_; child_node.f, child_node_id];

            node_accepted = true;

        else

            old_node = grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)};

            if old_node.is_open && child_candidate_g < old_node.g

                child_node.g = child_candidate_g;
                child_node.h = old_node.h;
                child_node.f = child_node.g + child_node.h;
                child_node.is_closed = 0;
                child_node.is_open = 1;

                grid_space_{child_node_id(1), child_node_id(2), child_node_id(3)} = child_node;
                UpdateOpenlist(child_node, child_node_id);

                node_accepted = true;
            end
        end

        if ~node_accepted
            continue;
        end

        %% -------------------------
        % Reeds-Shepp analytic expansion
        % --------------------------
        rs_counter = rs_counter + 1;

        distance_to_goal = hypot(child_node.x - params.task.xf, child_node.y - params.task.yf);

        if distance_to_goal <= params.ha.rs_trigger_distance || mod(rs_counter, params.ha.rs_try_interval) == 0

            rs_try = GenerateRsPathSeg(child_node);

            if ~isempty(rs_try) && IsRsPathValid(rs_try)
                completeness_flag = 1;
                complete_via_rs = true;
                rs_seg = rs_try;
                cur_node = child_node;
                break;
            end
        end

        %% -------------------------
        % Debug: closest node
        % --------------------------
        pos_error = hypot(child_node.x - params.task.xf, child_node.y - params.task.yf);
        theta_error = abs(atan2(sin(child_node.theta - params.task.thetaf), cos(child_node.theta - params.task.thetaf)));
        cost_val_candidate = pos_error + theta_error;

        if cost_val_candidate < cur_best_node_cost_val
            cur_best_node_cost_val = cost_val_candidate;
            cur_best_node = child_node;
        end
    end

    if completeness_flag
        break;
    end
end

%% =========================
% 4. Search result
% =========================
if ~completeness_flag

    if isempty(fail_reason)
        if isempty(openlist_)
            fail_reason = 'openlist 已耗尽，未找到可行解';
        else
            fail_reason = '未知搜索失败';
        end
    end

    fprintf('\nHybrid A* 搜索失败：%s\n', fail_reason);

    params.ha.success = 0;
    params.ha.fail_reason = fail_reason;
    return;
end

%% =========================
% 5. Backtracking Hybrid A*
% =========================
Traj.x = [];
Traj.y = [];
Traj.theta = [];
Traj.v = [];

while true

    path_seg = cur_node.from_parent;

    if isempty(path_seg)
        break;
    end

    nseg = numel(path_seg.x);

    Traj.x = [path_seg.x, Traj.x];
    Traj.y = [path_seg.y, Traj.y];
    Traj.theta = [path_seg.theta, Traj.theta];
    Traj.v = [ones(1, nseg) * cur_node.v, Traj.v];

    parent_id = cur_node.parent_id;
    cur_node = grid_space_{parent_id(1), parent_id(2), parent_id(3)};
end

x = Traj.x;
y = Traj.y;
theta = Traj.theta;
v_traj = Traj.v;

if isempty(x)
    error('Hybrid A*: reconstructed trajectory is empty.');
end

%% =========================
% 6. Append Reeds-Shepp path
% =========================
if complete_via_rs

    rs_dir = GetRsDirection(rs_seg);

    % rs_seg(1) 与 Hybrid A* 当前节点重合，不重复加入
    x = [x, rs_seg.x(2:end)];
    y = [y, rs_seg.y(2:end)];
    theta = [theta, rs_seg.theta(2:end)];
    v_traj = [v_traj, rs_dir(2:end)];

    fprintf('\nHybrid A*: Reeds-Shepp connection succeeded.\n');
end

%% =========================
% 7. Remove duplicated points
% =========================
keep = true(1, numel(x));

for i = 2:numel(x)

    pos_same = hypot(x(i) - x(i-1), y(i) - y(i-1)) < 1e-10;
    theta_same = abs(atan2(sin(theta(i) - theta(i-1)), cos(theta(i) - theta(i-1)))) < 1e-10;

    if pos_same && theta_same
        keep(i) = false;
    end
end

x = x(keep);
y = y(keep);
theta = theta(keep);
v_traj = v_traj(keep);

%% =========================
% 8. Unwrap heading
% =========================
for i = 2:numel(theta)

    while theta(i) - theta(i-1) > pi
        theta(i) = theta(i) - 2*pi;
    end

    while theta(i) - theta(i-1) < -pi
        theta(i) = theta(i) + 2*pi;
    end
end

%% =========================
% 9. Reconstruct steering angle
% =========================
phy_traj = ReconstructSteering(x, y, theta, v_traj);

%% =========================
% 10. Direction changes
% =========================
change_idx = find(v_traj(2:end) ~= v_traj(1:end-1)) + 1;
params.change = change_idx;

fprintf('\n========== Hybrid A* ==========\n');
fprintf('iterations          : %d\n', iter);
fprintf('search time         : %.3f s\n', toc(search_tic));
fprintf('trajectory points   : %d\n', numel(x));
fprintf('direction changes   : %d\n', numel(change_idx));
fprintf('completed by RS     : %d\n', complete_via_rs);
fprintf('final position error: %.6e m\n', hypot(x(end) - params.task.xf, y(end) - params.task.yf));
fprintf('final heading error : %.6e rad\n', abs(atan2(sin(theta(end) - params.task.thetaf), cos(theta(end) - params.task.thetaf))));
fprintf('================================\n\n');

%% =========================
% 11. Save result
% =========================
params.ha.x = x;
params.ha.y = y;
params.ha.theta = theta;
params.ha.v = v_traj;
params.ha.phy = phy_traj;
params.ha.success = 1;
params.ha.fail_reason = '';

success = 1;

end

%% ========================================================================
% Guiding-path remaining distance
% ========================================================================
function EstimateprogressAlongGuidingLine()

global params

s = zeros(1, length(params.guiding_path.x));

for ii = 2:length(params.guiding_path.x)
    s(ii) = s(ii-1) + hypot(params.guiding_path.x(ii) - params.guiding_path.x(ii-1), params.guiding_path.y(ii) - params.guiding_path.y(ii-1));
end

params.progress_s = s(end) - s;

end

%% ========================================================================
% Continuous state -> 3D Hybrid A* grid index
% ========================================================================
function idx = CalculateNodeIndex(node)

global params

params.ha_res_x = params.environment.xhorizon / params.ha.nx;
params.ha_res_y = params.environment.yhorizon / params.ha.ny;
params.ha_res_theta = 2*pi / params.ha.ntheta;

ind1 = ceil((node.x - params.environment.xmin) / params.ha_res_x) + 1;
ind2 = ceil((node.y - params.environment.ymin) / params.ha_res_y) + 1;
ind3 = ceil(RegulateAngle(node.theta) / params.ha_res_theta) + 1;

ind1 = max(1, min(ind1, params.ha.nx));
ind2 = max(1, min(ind2, params.ha.ny));
ind3 = max(1, min(ind3, params.ha.ntheta));

idx = [ind1, ind2, ind3];

end

%% ========================================================================
% Motion primitives
% ========================================================================
function expansion_pattern = SpecifySamplePattern()

global params

phy_list = linspace(-params.vehicle.phy_max, params.vehicle.phy_max, params.ha.num_phy_ha);

expansion_pattern = [];

for ii = 1:params.ha.num_phy_ha
    expansion_pattern = [expansion_pattern; 1, phy_list(ii)];
    expansion_pattern = [expansion_pattern; -1, phy_list(ii)];
end

end

%% ========================================================================
% Open list
% ========================================================================
function [cur_node, cur_node_id] = ExtractMinFNodeFromOpenlist()

global grid_space_ openlist_

[~, id] = min(openlist_(:,1));

cur_node_id = openlist_(id,2:4);
cur_node = grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)};

openlist_(id,:) = [];

grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_closed = 1;
grid_space_{cur_node_id(1), cur_node_id(2), cur_node_id(3)}.is_open = 0;

end

function UpdateOpenlist(child_node, child_node_id)

global openlist_

mask = openlist_(:,2) == child_node_id(1) & openlist_(:,3) == child_node_id(2) & openlist_(:,4) == child_node_id(3);

openlist_(mask,:) = [];
openlist_ = [openlist_; child_node.f, child_node_id];

end

%% ========================================================================
% Hybrid A* primitive collision
% ========================================================================
function is_valid = IsPathValid(path_seg)

is_valid = IsVehicleBodyValid(path_seg.x, path_seg.y, path_seg.theta);

end

function is_valid = IsVehicleBodyValid(x, y, theta)

is_valid = false;

for kk = 1:numel(x)

    if ~IsVehiclePoseValid(x(kk), y(kk), theta(kk))
        return;
    end
end

is_valid = true;

end

function is_valid = IsVehiclePoseValid(x, y, theta)

global params

is_valid = false;

body_poly = GetVehicleRectangle(x, y, theta);

if any(body_poly(:,1) > params.environment.xmax) || any(body_poly(:,1) < params.environment.xmin) || ...
   any(body_poly(:,2) > params.environment.ymax) || any(body_poly(:,2) < params.environment.ymin)
    return;
end

for ii = 1:params.environment.num_obs

    obs_poly = [params.environment.obs(ii).x(:), params.environment.obs(ii).y(:)];

    if SAT_PolygonCollision(body_poly, obs_poly)
        return;
    end
end

is_valid = true;

end

%% ========================================================================
% Reeds-Shepp analytic expansion
% ========================================================================
function rs_seg = GenerateRsPathSeg(node)

global params

rs_seg = [];

start_pose = [node.x, node.y, node.theta];
goal_pose = [params.task.xf, params.task.yf, params.task.thetaf];

Rmin = params.vehicle.lw / tan(params.vehicle.phy_max);

reeds_conn = robotics.ReedsSheppConnection('MinTurningRadius', Rmin);
reeds_conn.ReverseCost = params.ha.rs_reverse_cost;

[path_obj, ~] = connect(reeds_conn, start_pose, goal_pose);

if isempty(path_obj) || isempty(path_obj{1})
    return;
end

path_length = path_obj{1}.Length;

if ~isfinite(path_length) || path_length <= 0
    return;
end

sample_s = 0:params.ha.rs_sample_ds:path_length;

if isempty(sample_s) || sample_s(end) < path_length - 1e-12
    sample_s = [sample_s, path_length];
end

poses = interpolate(path_obj{1}, sample_s);

if isempty(poses)
    return;
end

rs_seg.x = poses(:,1).';
rs_seg.y = poses(:,2).';
rs_seg.theta = poses(:,3).';

% 强制最后一个点为精确任务终点，消除 interpolate 数值误差
rs_seg.x(end) = params.task.xf;
rs_seg.y(end) = params.task.yf;
rs_seg.theta(end) = params.task.thetaf;

end

function is_valid = IsRsPathValid(rs_seg)

is_valid = false;

if isempty(rs_seg) || numel(rs_seg.x) < 2
    return;
end

for i = 1:numel(rs_seg.x)

    if ~IsVehiclePoseValid(rs_seg.x(i), rs_seg.y(i), rs_seg.theta(i))
        return;
    end
end

is_valid = true;

end

%% ========================================================================
% Determine forward / reverse direction on RS path
% ========================================================================
function dir = GetRsDirection(rs_seg)

N = numel(rs_seg.x);
dir = ones(1, N);

if N <= 1
    return;
end

for i = 1:N-1

    dx = rs_seg.x(i+1) - rs_seg.x(i);
    dy = rs_seg.y(i+1) - rs_seg.y(i);

    projection = dx*cos(rs_seg.theta(i)) + dy*sin(rs_seg.theta(i));

    if projection > 1e-8
        dir(i) = 1;
    elseif projection < -1e-8
        dir(i) = -1;
    elseif i > 1
        dir(i) = dir(i-1);
    end
end

dir(end) = dir(end-1);

end

%% ========================================================================
% Reconstruct steering angle
% ========================================================================
function phy = ReconstructSteering(x, y, theta, dir)

global params

N = numel(x);
phy = zeros(1, N);

for i = 1:N-1

    dx = x(i+1) - x(i);
    dy = y(i+1) - y(i);
    ds = hypot(dx, dy);

    dtheta = atan2(sin(theta(i+1) - theta(i)), cos(theta(i+1) - theta(i)));

    if ds < 1e-8
        phy(i) = 0;
        continue;
    end

    motion_dir = sign(dir(i));

    if motion_dir == 0
        motion_dir = 1;
    end

    signed_ds = motion_dir * ds;
    kappa = dtheta / signed_ds;

    phy(i) = atan(kappa * params.vehicle.lw);

    % 防止数值离散导致略微超过最大转角
    phy(i) = max(-params.vehicle.phy_max, min(params.vehicle.phy_max, phy(i)));
end

if N >= 2
    phy(N) = phy(N-1);
end

end

%% ========================================================================
% SAT collision
% ========================================================================
function is_collide = SAT_PolygonCollision(poly1, poly2)

is_collide = true;

axes = [GetAxesFromPolygon(poly1); GetAxesFromPolygon(poly2)];

for k = 1:size(axes,1)

    axis_k = axes(k,:);
    nrm = norm(axis_k);

    if nrm < 1e-12
        continue;
    end

    axis_k = axis_k / nrm;

    proj1 = poly1 * axis_k';
    proj2 = poly2 * axis_k';

    if max(proj1) < min(proj2) || max(proj2) < min(proj1)
        is_collide = false;
        return;
    end
end

end

function axes = GetAxesFromPolygon(poly)

n = size(poly,1);
axes = zeros(n,2);

for i = 1:n

    p1 = poly(i,:);

    if i < n
        p2 = poly(i+1,:);
    else
        p2 = poly(1,:);
    end

    edge = p2 - p1;
    axes(i,:) = [-edge(2), edge(1)];
end

end

%% ========================================================================
% Heuristic
% ========================================================================
function val = CalculateH(child_node)

global params

list = hypot(params.guiding_path.x - child_node.x, params.guiding_path.y - child_node.y);

[distance_to_guiding_path, ind] = min(list);

val = params.ha.multiplier_on_heuristics * ...
      (params.progress_s(ind) + 0.1 * distance_to_guiding_path);

end

%% ========================================================================
% Debug figure
% ========================================================================
function InitHADebugFigure()

global params

figure('Name', 'Hybrid A* Debug View', 'Color', 'w');
clf;
hold on;
box on;
grid on;
axis equal;

xlabel('x');
ylabel('y');
title('Hybrid A* Search Debug');

xlim([params.environment.xmin, params.environment.xmax]);
ylim([params.environment.ymin, params.environment.ymax]);

for ii = 1:params.environment.num_obs
    fill(params.environment.obs(ii).x, params.environment.obs(ii).y, [0.7 0.7 0.7], ...
        'EdgeColor', 'k', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
end

plot(params.task.x0, params.task.y0, 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
plot(params.task.xf, params.task.yf, 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Goal');

if isfield(params, 'guiding_path') && isfield(params.guiding_path, 'x') && ~isempty(params.guiding_path.x)
    plot(params.guiding_path.x, params.guiding_path.y, 'b--', 'LineWidth', 1.0, 'DisplayName', 'Guiding path');
end

legend('Location', 'best');

end

function UpdateHADebugFigureSimple(iter, cur_node, cur_best_node)

global params ha_debug_

cla;
hold on;
box on;
grid on;
axis equal;

xlim([params.environment.xmin, params.environment.xmax]);
ylim([params.environment.ymin, params.environment.ymax]);

for ii = 1:params.environment.num_obs
    fill(params.environment.obs(ii).x, params.environment.obs(ii).y, [0.7 0.7 0.7], ...
        'EdgeColor', 'k', 'FaceAlpha', 0.5);
end

plot(params.guiding_path.x, params.guiding_path.y, 'b--', 'LineWidth', 1);
plot(params.task.x0, params.task.y0, 'go', 'MarkerSize', 8, 'LineWidth', 2);
plot(params.task.xf, params.task.yf, 'ro', 'MarkerSize', 8, 'LineWidth', 2);

if ~isempty(ha_debug_.last_cur_path_x)
    plot(ha_debug_.last_cur_path_x, ha_debug_.last_cur_path_y, 'k-', 'LineWidth', 1.5);
end

plot(cur_node.x, cur_node.y, 'mo', 'MarkerSize', 6, 'LineWidth', 1.5);
plot(cur_best_node.x, cur_best_node.y, 'co', 'MarkerSize', 6, 'LineWidth', 1.5);

title(sprintf('Hybrid A*: iter = %d', iter));

end

%% ========================================================================
% Trace one node back to start
% ========================================================================
function [x_path, y_path] = TraceNodePath(node)

global grid_space_

x_path = [];
y_path = [];
cur_node = node;

while true

    if ~isfield(cur_node, 'from_parent') || isempty(cur_node.from_parent)
        break;
    end

    seg = cur_node.from_parent;

    x_path = [seg.x, x_path];
    y_path = [seg.y, y_path];

    pid = cur_node.parent_id;

    if any(pid < 1)
        break;
    end

    cur_node = grid_space_{pid(1), pid(2), pid(3)};
end

end
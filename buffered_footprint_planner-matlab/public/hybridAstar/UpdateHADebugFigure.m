%%  （更新混合A*显示界面）
function UpdateHADebugFigure(iter, cur_node, cur_best_node, completeness_flag)
global params grid_space_ openlist_ ha_debug_

fig = findobj('Type', 'figure', 'Name', 'Hybrid A* Debug View');
if isempty(fig)
    fig = figure('Name', 'Hybrid A* Debug View', 'Color', 'w');
else
    figure(fig);
end
clf;
hold on; box on; grid on; axis equal;

xlabel('x');
ylabel('y');

if completeness_flag
    status_str = 'SUCCESS';
else
    status_str = 'SEARCHING';
end

title(sprintf('Hybrid A* Debug | iter = %d | open = %d | %s', ...
    iter, size(openlist_,1), status_str));

xlim([params.environment.xmin, params.environment.xmax]);
ylim([params.environment.ymin, params.environment.ymax]);

% ==================== 1) 障碍物 ====================
hObs = gobjects(0);
for ii = 1:params.environment.num_obs
    h = fill(params.environment.obs(ii).x, params.environment.obs(ii).y, ...
        [0.7 0.7 0.7], ...
        'EdgeColor', 'k', ...
        'FaceAlpha', 0.5, ...
        'HandleVisibility', 'off');   % 先都关掉
    if ii == 1
        hObs = h;                     % 只拿第一个障碍物做图例
        set(hObs, 'HandleVisibility', 'on');
    end
end

% ==================== 2) 引导线 ====================
hGuide = gobjects(1);
if isfield(params, 'guiding_path') && isfield(params.guiding_path, 'x') && ~isempty(params.guiding_path.x)
    hGuide = plot(params.guiding_path.x, params.guiding_path.y, 'b--', ...
        'LineWidth', 1.0, ...
        'DisplayName', 'Guiding path');
else
    hGuide = plot(nan, nan, 'b--', ...
        'LineWidth', 1.0, ...
        'DisplayName', 'Guiding path');
end

% ==================== 3) open / closed 点 ====================
open_x = [];
open_y = [];
closed_x = [];
closed_y = [];

for ix = 1:params.ha.nx
    for iy = 1:params.ha.ny
        for it = 1:params.ha.ntheta
            node = grid_space_{ix, iy, it};
            if isempty(node)
                continue;
            end

            if isfield(node, 'is_closed') && node.is_closed == 1
                closed_x(end+1) = node.x; %#ok<AGROW>
                closed_y(end+1) = node.y; %#ok<AGROW>
            elseif isfield(node, 'is_open') && node.is_open == 1
                open_x(end+1) = node.x; %#ok<AGROW>
                open_y(end+1) = node.y; %#ok<AGROW>
            end
        end
    end
end

if ~isempty(closed_x)
    hClosed = plot(closed_x, closed_y, '.', ...
        'Color', [0.85 0.85 0.85], ...
        'MarkerSize', 4, ...
        'DisplayName', 'Closed nodes');
else
    hClosed = plot(nan, nan, '.', ...
        'Color', [0.85 0.85 0.85], ...
        'MarkerSize', 4, ...
        'DisplayName', 'Closed nodes');
end

if ~isempty(open_x)
    hOpen = plot(open_x, open_y, '.', ...
        'Color', [1 0.6 0], ...
        'MarkerSize', 5, ...
        'DisplayName', 'Open nodes');
else
    hOpen = plot(nan, nan, '.', ...
        'Color', [1 0.6 0], ...
        'MarkerSize', 5, ...
        'DisplayName', 'Open nodes');
end

% ==================== 4) 当前节点到起点的回溯路径 ====================
if isfield(ha_debug_, 'last_cur_path_x') && ~isempty(ha_debug_.last_cur_path_x)
    hCurPath = plot(ha_debug_.last_cur_path_x, ha_debug_.last_cur_path_y, 'k-', ...
        'LineWidth', 2, ...
        'DisplayName', 'Current best path');
else
    hCurPath = plot(nan, nan, 'k-', ...
        'LineWidth', 2, ...
        'DisplayName', 'Current best path');
end

% ==================== 5) 最近失败候选框（不进 legend） ====================
if isfield(ha_debug_, 'last_invalid') && ~isempty(ha_debug_.last_invalid)
    for j = 1:numel(ha_debug_.last_invalid)
        dbg = ha_debug_.last_invalid{j};

        % 真实车体框：不进 legend
        DrawVehicleBox(dbg.x, dbg.y, dbg.theta, 'r-', 1.0);
        hh = get(gca, 'Children');
        set(hh(1), 'HandleVisibility', 'off');

        % 扫掠框：不进 legend
        local_dt_dbg = GetAdaptiveSimuDuration(dbg.x, dbg.y);
        DrawSweepBox(dbg.x, dbg.y, dbg.theta, dbg.phy, dbg.dir, local_dt_dbg, 'r--', 0.8);
        hh = get(gca, 'Children');
        n_new = min(numel(hh), 10);   % 保险起见，关掉最近若干新对象的 legend
        set(hh(1:n_new), 'HandleVisibility', 'off');
    end
end

% ==================== 6) 当前节点：点 + 车体框 + 扫掠框 ====================
if ~isempty(cur_node)
    hCur = plot(cur_node.x, cur_node.y, 'mo', ...
        'MarkerSize', 8, ...
        'LineWidth', 2, ...
        'DisplayName', 'Current node');

    DrawVehicleBox(cur_node.x, cur_node.y, cur_node.theta, 'm-', 2.0);
    hh = get(gca, 'Children');
    set(hh(1), 'HandleVisibility', 'off');  % 当前车体框不单独进 legend

    local_dt_cur = GetAdaptiveSimuDuration(cur_node.x, cur_node.y);
    DrawSweepBox(cur_node.x, cur_node.y, cur_node.theta, cur_node.phy, cur_node.v, local_dt_cur, 'm--', 1.5);
    hh = get(gca, 'Children');
    n_new = min(numel(hh), 10);
    set(hh(1:n_new), 'HandleVisibility', 'off'); % 当前扫掠框不单独进 legend
else
    hCur = plot(nan, nan, 'mo', ...
        'MarkerSize', 8, ...
        'LineWidth', 2, ...
        'DisplayName', 'Current node');
end

% ==================== 7) 当前 best 节点 ====================
if ~isempty(cur_best_node)
    hBest = plot(cur_best_node.x, cur_best_node.y, 'co', ...
        'MarkerSize', 8, ...
        'LineWidth', 2, ...
        'DisplayName', 'Best-so-far');
else
    hBest = plot(nan, nan, 'co', ...
        'MarkerSize', 8, ...
        'LineWidth', 2, ...
        'DisplayName', 'Best-so-far');
end

% ==================== 8) 起点终点 ====================
if isfield(params, 'task')
    x0 = params.task.x0; y0 = params.task.y0;
    xf = params.task.xf; yf = params.task.yf;
else
    % 兼容旧写法
    x0 = params.start.x;
    y0 = params.start.y;
    xf = params.goal.x;
    yf = params.goal.y;
end

hStart = plot(x0, y0, 'go', ...
    'MarkerSize', 10, ...
    'LineWidth', 2, ...
    'DisplayName', 'Start');

hGoal = plot(xf, yf, 'ro', ...
    'MarkerSize', 10, ...
    'LineWidth', 2, ...
    'DisplayName', 'Goal');


legend_handles = [hObs, hGuide, hClosed, hOpen, hCurPath, hCur, hBest, hStart, hGoal];
legend_labels  = {'Obstacle', 'Guiding path', 'Closed nodes', 'Open nodes', ...
                  'Current best path', 'Current node', 'Best-so-far', 'Start', 'Goal'};

legend(legend_handles, legend_labels, 'Location', 'eastoutside');
end

%%
function DrawVehicleBox(x, y, theta, styleStr, lineWidth)
global params

AX = x + params.vehicle.LF * cos(theta) - params.vehicle.hlb * sin(theta);
AY = y + params.vehicle.LF * sin(theta) + params.vehicle.hlb * cos(theta);

BX = x + params.vehicle.LF * cos(theta) + params.vehicle.hlb * sin(theta);
BY = y + params.vehicle.LF * sin(theta) - params.vehicle.hlb * cos(theta);

CX = x - params.vehicle.lr * cos(theta) + params.vehicle.hlb * sin(theta);
CY = y - params.vehicle.lr * sin(theta) - params.vehicle.hlb * cos(theta);

DX = x - params.vehicle.lr * cos(theta) - params.vehicle.hlb * sin(theta);
DY = y - params.vehicle.lr * sin(theta) + params.vehicle.hlb * cos(theta);

plot([AX BX CX DX AX], [AY BY CY DY AY], styleStr, 'LineWidth', lineWidth);
end
%%
function DrawSweepBox(x, y, theta, phy, dir, local_dt, styleStr, lineWidth)
global params

k = tan(phy) / params.vehicle.lw;
step_len = dir * local_dt;

[a, b, c, d] = EstimateScaledAABB(step_len, k);

[AX, AY, BX, BY, CX, CY, DX, DY] = ddd2(x, y, theta, a, b, c, d);

for ii = 1:numel(AX)
    plot([AX(ii) BX(ii) CX(ii) DX(ii) AX(ii)], ...
         [AY(ii) BY(ii) CY(ii) DY(ii) AY(ii)], ...
         styleStr, 'LineWidth', lineWidth);
end
end
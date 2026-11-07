function PlotTrueVehicleSweptAreaOnly()
% Plot true vehicle swept area, obstacles, and start/end pose arrows.
% This view does not read embodied-footprint box variables.

global params

assert(isfield(params, 'ef'), 'params.ef is missing. Run LoadEFOptimumAndRefine first.');
assert(isfield(params, 'vehicle'), 'params.vehicle is missing.');
assert(isfield(params, 'environment'), 'params.environment is missing.');

[x, y, th, v] = loadSweptPose();
if numel(x) < 2
    error('Too few trajectory points to build swept area.');
end

lf  = params.vehicle.lf;
lw  = params.vehicle.lw;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

fig = figure('Name', 'True Vehicle Swept Area', 'Color', 'w');
ax = axes(fig);
hold(ax, 'on');
box(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');

if isfield(params.environment, 'xmin') && isfield(params.environment, 'xmax') && ...
   isfield(params.environment, 'ymin') && isfield(params.environment, 'ymax')
    axis(ax, [params.environment.xmin, params.environment.xmax, ...
              params.environment.ymin, params.environment.ymax]);
end

xlabel(ax, 'x / m');
ylabel(ax, 'y / m');
title(ax, 'True Vehicle Swept Area');

obs_face = [0.72 0.72 0.70];
obs_edge = [0.38 0.38 0.36];
path_col = [0.18 0.18 0.18];
forward_col = [0.15 0.40 0.95];
reverse_col = [0.90 0.15 0.15];
start_col = [0.18 0.42 0.32];
goal_col = [0.35 0.27 0.48];

if isfield(params.environment, 'obs') && ~isempty(params.environment.obs)
    for ii = 1:numel(params.environment.obs)
        fill(ax, params.environment.obs(ii).x, params.environment.obs(ii).y, obs_face, ...
            'EdgeColor', obs_edge, ...
            'LineWidth', 0.9, ...
            'HandleVisibility', 'off');
    end
end

%% ========== 构造真实车辆前进/后退扫掠区域 ==========

swept_forward = polyshape();
swept_reverse = polyshape();

eps_dir = 1e-10;

for k = 1:numel(x)

    [XB, YB] = bodyRectCorners( ...
        x(k), y(k), th(k), ...
        params.vehicle.lf, ...
        params.vehicle.lw, ...
        params.vehicle.lr, ...
        params.vehicle.hlb);

    bodyPoly = polyshape(XB, YB);

    if v(k) > eps_dir

        swept_forward = union( ...
            swept_forward, ...
            bodyPoly);

    elseif v(k) < -eps_dir

        swept_reverse = union( ...
            swept_reverse, ...
            bodyPoly);

    else
        % 零速点不单独分类
    end

end


%% ========== 绘制真实扫掠区域 ==========

h_forward = plot( ...
    swept_forward, ...
    'FaceColor', forward_col, ...
    'FaceAlpha', 0.30, ...
    'EdgeColor', 'none');

h_reverse = plot( ...
    swept_reverse, ...
    'FaceColor', reverse_col, ...
    'FaceAlpha', 0.30, ...
    'EdgeColor', 'none');

h_path = plot(ax, x, y, '-', ...
    'Color', path_col, ...
    'LineWidth', 1.2);

h_start = drawPoseArrow(ax, x(1), y(1), th(1), start_col, 'Start');
h_goal = drawPoseArrow(ax, x(end), y(end), th(end), goal_col, 'Goal');

legend(ax, [h_forward, h_reverse, h_path, h_start, h_goal], ...
    {'Forward swept area', 'Reverse swept area', 'Optimized trajectory', ...
     'Start pose', 'Goal pose'}, ...
    'Location', 'eastoutside');

fprintf('\n================ True Swept Area Plot Finished ================\n');
fprintf('Dense swept-pose count         : %d\n', numel(x));
fprintf('No EF variables were read for this view.\n');
fprintf('===============================================================\n\n');

drawnow;
SaveTaskFigure(gcf, 'TrueBody');

end

function [x, y, th, v] = loadSweptPose()
global params

use_enriched = isfield(params.ef, 'enriched_x') && ~isempty(params.ef.enriched_x) && ...
               isfield(params.ef, 'enriched_y') && ~isempty(params.ef.enriched_y) && ...
               isfield(params.ef, 'enriched_theta') && ~isempty(params.ef.enriched_theta);

if use_enriched
    x = params.ef.enriched_x(:);
    y = params.ef.enriched_y(:);
    th = params.ef.enriched_theta(:);
else
    x = params.ef.x(:);
    y = params.ef.y(:);
    th = params.ef.theta(:);
end

n = min([numel(x), numel(y), numel(th)]);
x = x(1:n);
y = y(1:n);
th = th(1:n);

if use_enriched && isfield(params.ef, 'enriched_v') && ~isempty(params.ef.enriched_v)
    v = params.ef.enriched_v(:);
elseif isfield(params.ef, 'v') && ~isempty(params.ef.v)
    v0 = params.ef.v(:);
    if numel(v0) == n
        v = v0;
    else
        idx = round(linspace(1, numel(v0), n));
        v = v0(idx);
    end
else
    v = estimateDirectionFromPose(x, y, th);
end

if isempty(v)
    v = estimateDirectionFromPose(x, y, th);
elseif numel(v) < n
    v(end+1:n) = v(end);
else
    v = v(1:n);
end
end

function [X, Y] = bodyRectCorners(x, y, theta, lf, lw, lr, hlb)
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

X = [AX BX CX DX AX];
Y = [AY BY CY DY AY];
end

function h_arrow = drawPoseArrow(ax, x, y, theta, color, label_text)
global params

arrow_len = 1.1;
if isfield(params, 'vehicle') && isfield(params.vehicle, 'lw')
    arrow_len = 0.55 * params.vehicle.lw;
end

h_arrow = quiver(ax, x, y, arrow_len * cos(theta), arrow_len * sin(theta), 0, ...
    'Color', color, ...
    'LineWidth', 2.0, ...
    'MaxHeadSize', 0.8);
plot(ax, x, y, 'o', ...
    'MarkerSize', 5, ...
    'MarkerFaceColor', color, ...
    'MarkerEdgeColor', 'w', ...
    'LineWidth', 0.8, ...
    'HandleVisibility', 'off');
text(ax, x, y, ['  ', label_text], ...
    'Color', color, ...
    'FontSize', 10, ...
    'FontWeight', 'bold', ...
    'VerticalAlignment', 'bottom', ...
    'HandleVisibility', 'off');
end

function v_est = estimateDirectionFromPose(x, y, th)
n = numel(x);
v_est = zeros(n, 1);

for k = 1:n-1
    dx = x(k+1) - x(k);
    dy = y(k+1) - y(k);
    proj = dx * cos(th(k)) + dy * sin(th(k));

    if proj > 1e-10
        v_est(k) = 1;
    elseif proj < -1e-10
        v_est(k) = -1;
    else
        v_est(k) = 0;
    end
end

if n >= 2
    v_est(n) = v_est(n-1);
end
end

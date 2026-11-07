function DrawEnvironment(matfile)
% DrawEnvironment 读取并绘制任务环境
% 用法:
%   DrawEnvironment('6.mat')
%   DrawEnvironment("D:\desktop\...\6.mat")

clc;
close all;
global params

%% ===== 读取数据 =====
S = load(matfile);

required_vars = {'obs','x0','y0','theta0','xf','yf','thetaf'};
for i = 1:length(required_vars)
    if ~isfield(S, required_vars{i})
        error('文件中缺少变量: %s', required_vars{i});
    end
end

obs    = S.obs;
x0     = S.x0;
y0     = S.y0;
theta0 = S.theta0;
xf     = S.xf;
yf     = S.yf;
thetaf = S.thetaf;

%% ===== 检查环境参数 =====
if ~isfield(params, 'environment') || ...
   ~isfield(params.environment, 'xmin') || ...
   ~isfield(params.environment, 'xmax') || ...
   ~isfield(params.environment, 'ymin') || ...
   ~isfield(params.environment, 'ymax')
    error('global params.environment.xmin/xmax/ymin/ymax 未正确设置。');
end

xmin = params.environment.xmin;
xmax = params.environment.xmax;
ymin = params.environment.ymin;
ymax = params.environment.ymax;

env_dx = xmax - xmin;
env_dy = ymax - ymin;

if env_dx <= 0 || env_dy <= 0
    error('环境范围非法，请检查 params.environment 的 xmin/xmax/ymin/ymax。');
end

%% ===== 开始绘图 =====
figure('Name','Environment Viewer','Color','w');
set(gcf, 'outerposition', get(0,'screensize'));
hold on; box on; grid minor; axis equal;

axis([xmin, xmax, ymin, ymax]);

xlabel('x / m');
ylabel('y / m');
title('Task Environment');

%% ===== 绘制障碍物 =====
h_obs = gobjects(length(obs),1);

for i = 1:length(obs)
    x = obs(i).x(:)';
    y = obs(i).y(:)';

    if isempty(x) || isempty(y)
        warning('障碍物 %d 为空，已跳过。', i);
        continue;
    end

    % 检查闭合
    is_closed = (abs(x(1)-x(end)) < 1e-10) && (abs(y(1)-y(end)) < 1e-10);

    % 若未闭合，则自动补闭合
    if ~is_closed
        x_plot = [x, x(1)];
        y_plot = [y, y(1)];
        nv = length(x);
    else
        x_plot = x;
        y_plot = y;
        nv = length(x) - 1;
    end

    % 填充障碍物
    h_obs(i) = fill(x_plot, y_plot, [0.75 0.75 0.75], ...
        'EdgeColor', 'k', 'LineWidth', 1.2);

    % 障碍物中心编号
    if is_closed
        cx = mean(x(1:end-1));
        cy = mean(y(1:end-1));
    else
        cx = mean(x);
        cy = mean(y);
    end

    text(cx, cy, sprintf('Obs %d', i), ...
        'Color', 'b', ...
        'FontSize', 12, ...
        'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');

    % 顶点编号
    for j = 1:nv
        plot(x(j), y(j), 'ko', ...
            'MarkerFaceColor', 'k', ...
            'MarkerSize', 5, ...
            'LineWidth', 1.0);

        text(x(j), y(j), sprintf('  %d', j), ...
            'Color', 'k', ...
            'FontSize', 10, ...
            'FontWeight', 'bold');
    end
end

%% ===== 绘制起点和终点 =====
h_start = plot(x0, y0, 'go', ...
    'MarkerFaceColor', 'g', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.5);

h_goal = plot(xf, yf, 'mo', ...
    'MarkerFaceColor', 'm', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.5);

%% ===== 绘制朝向箭头 =====
L = 0.05 * max(env_dx, env_dy);   % 与地图参数一致
L = max(L, 1.0);

if exist('Arrow', 'file') == 2
    Arrow([x0, y0], [x0 + L*cos(theta0), y0 + L*sin(theta0)], ...
        'Length', 10, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 1);
    Arrow([xf, yf], [xf + L*cos(thetaf), yf + L*sin(thetaf)], ...
        'Length', 10, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 1);
else
    quiver(x0, y0, L*cos(theta0), L*sin(theta0), 0, ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 1);

    quiver(xf, yf, L*cos(thetaf), L*sin(thetaf), 0, ...
        'Color', 'm', 'LineWidth', 2, 'MaxHeadSize', 1);
end

text(x0, y0, '  Start', ...
    'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

text(xf, yf, '  Goal', ...
    'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');

%% ===== 坐标轴与图例 =====
set(gca, 'FontWeight', 'bold', 'FontSize', 16, 'FontName', 'Arial');

valid_obs = h_obs(isgraphics(h_obs));
if ~isempty(valid_obs)
    legend([valid_obs(1), h_start, h_goal], ...
        {'Obstacle','Start','Goal'}, ...
        'Location', 'best');
else
    legend([h_start, h_goal], ...
        {'Start','Goal'}, ...
        'Location', 'best');
end

drawnow;

%% ===== 输出基本信息 =====
fprintf('================ Environment Info ================\n');
fprintf('地图范围: x=[%.4f, %.4f], y=[%.4f, %.4f]\n', xmin, xmax, ymin, ymax);
fprintf('障碍物数量: %d\n', length(obs));
fprintf('起点: x=%.4f, y=%.4f, theta=%.4f\n', x0, y0, theta0);
fprintf('终点: x=%.4f, y=%.4f, theta=%.4f\n', xf, yf, thetaf);

for i = 1:length(obs)
    x = obs(i).x(:)';
    y = obs(i).y(:)';
    is_closed = (abs(x(1)-x(end)) < 1e-10) && (abs(y(1)-y(end)) < 1e-10);
    if is_closed
        nv = length(x) - 1;
    else
        nv = length(x);
    end
    fprintf('障碍物 %d: 顶点数 = %d, 闭合 = %d\n', i, nv, is_closed);
end
fprintf('==================================================\n');

end
function modify_target(file_path)
%MODIFY_TARGET 用鼠标修改已有任务文件中的起点和终点位姿。
%
% 每个位姿通过两次鼠标左键点击确定：
%   第一次：车辆后轴中心位置；
%   第二次：车辆朝向参考点。
%
% 原任务文件中的障碍物及其他字段保持不变，仅覆盖：
%   x0, y0, theta0, xf, yf, thetaf

global params

%% ===== 读取并检查任务文件 =====
if nargin < 1 || isempty(file_path)
    error('modify_target:MissingFilePath', '必须提供任务文件路径。');
end

if exist(file_path, 'file') ~= 2
    error('modify_target:FileNotFound', ...
        'Task file does not exist: %s', file_path);
end

data = load(file_path);
required_vars = {'obs', 'x0', 'y0', 'theta0', 'xf', 'yf', 'thetaf'};
for k = 1:numel(required_vars)
    if ~isfield(data, required_vars{k})
        error('modify_target:MissingVariable', ...
            '任务文件中缺少变量: %s', required_vars{k});
    end
end

%% ===== 创建交互窗口 =====
fig = figure('Name', 'Modify Start and Goal Poses', ...
             'Color', 'w', ...
             'NumberTitle', 'off');
set(fig, 'outerposition', get(0, 'screensize'));

hold on;
box on;
grid minor;
axis equal;
xlabel('x / m');
ylabel('y / m');
title({'Modify Start and Goal Poses', ...
       '每个位姿点击两次：后轴中心 + 朝向参考点'});

[xmin, xmax, ymin, ymax] = determineAxisLimits(data, params);
axis([xmin, xmax, ymin, ymax]);

env_dx = xmax - xmin;
env_dy = ymax - ymin;
arrow_len = 0.05 * max(env_dx, env_dy);

%% ===== 绘制已有障碍物 =====
for i = 1:numel(data.obs)
    obs_x = data.obs(i).x(:).';
    obs_y = data.obs(i).y(:).';

    if isempty(obs_x) || isempty(obs_y)
        continue;
    end

    fill(obs_x, obs_y, [0.75 0.75 0.75], ...
        'EdgeColor', 'k', 'LineWidth', 1.2);

    if numel(obs_x) >= 2 && ...
       abs(obs_x(1) - obs_x(end)) < 1e-10 && ...
       abs(obs_y(1) - obs_y(end)) < 1e-10
        center_x = mean(obs_x(1:end-1));
        center_y = mean(obs_y(1:end-1));
    else
        center_x = mean(obs_x);
        center_y = mean(obs_y);
    end

    text(center_x, center_y, sprintf('Obs %d', i), ...
        'Color', 'b', 'FontSize', 11, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
end

%% ===== 绘制原起终点位姿 =====
drawPose(data.x0, data.y0, data.theta0, arrow_len, ...
    [0.35 0.65 0.35], 'Old Start', '--');
drawPose(data.xf, data.yf, data.thetaf, arrow_len, ...
    [0.70 0.40 0.70], 'Old Goal', '--');
drawnow;

fprintf('\n==============================\n');
fprintf('请在图窗中重新指定起点和终点位姿。\n');
fprintf('每个位姿点击两次：\n');
fprintf('  第一次：车辆后轴中心位置\n');
fprintf('  第二次：车辆朝向参考点\n');

%% ===== 鼠标指定新起点 =====
fprintf('\n请选择新起点位姿：\n');
fprintf('第1次点击：起点后轴中心\n');
fprintf('第2次点击：起点朝向参考点\n');

[data.x0, data.y0, data.theta0] = getPoseByTwoClicks(fig, 'Start');
drawPose(data.x0, data.y0, data.theta0, arrow_len, ...
    [0 0.65 0], 'Start', '-');
drawnow;

%% ===== 鼠标指定新终点 =====
fprintf('\n请选择新终点位姿：\n');
fprintf('第1次点击：终点后轴中心\n');
fprintf('第2次点击：终点朝向参考点\n');

[data.xf, data.yf, data.thetaf] = getPoseByTwoClicks(fig, 'Goal');
drawPose(data.xf, data.yf, data.thetaf, arrow_len, ...
    [0.75 0 0.75], 'Goal', '-');
drawnow;

%% ===== 覆盖保存，仅更新位姿字段 =====
save(file_path, '-struct', 'data');

fprintf('\n任务文件已更新：\n%s\n', file_path);
fprintf('Start: x = %.4f, y = %.4f, theta = %.4f rad\n', ...
    data.x0, data.y0, data.theta0);
fprintf('Goal : x = %.4f, y = %.4f, theta = %.4f rad\n', ...
    data.xf, data.yf, data.thetaf);

end


function [x, y, theta] = getPoseByTwoClicks(fig, pose_name)
% 第一次点击确定位置，第二次点击确定朝向。

figure(fig);

while true
    [x1, y1, button1] = ginput(1);
    if isempty(button1) || button1 ~= 1
        fprintf('%s 第一个点无效，请使用鼠标左键重新点击。\n', pose_name);
        continue;
    end

    center_marker = plot(x1, y1, 'ks', ...
        'MarkerFaceColor', 'y', 'MarkerSize', 6, 'LineWidth', 1.0);
    center_label = text(x1, y1, sprintf('  %s-Center', pose_name), ...
        'Color', 'k', 'FontSize', 10, 'FontWeight', 'bold');
    drawnow;

    [x2, y2, button2] = ginput(1);
    if isempty(button2) || button2 ~= 1
        deleteIfValid(center_marker);
        deleteIfValid(center_label);
        fprintf('%s 第二个点无效，请重新选择该位姿。\n', pose_name);
        continue;
    end

    dx = x2 - x1;
    dy = y2 - y1;
    if hypot(dx, dy) < 1e-10
        deleteIfValid(center_marker);
        deleteIfValid(center_label);
        fprintf('%s 两个点过近，无法确定方向，请重新选择。\n', pose_name);
        continue;
    end

    plot([x1, x2], [y1, y2], 'k-.', 'LineWidth', 1.2);
    plot(x2, y2, 'k.', 'MarkerSize', 15);

    x = x1;
    y = y1;
    theta = atan2(dy, dx);

    fprintf('%s 位姿已确定：x = %.4f, y = %.4f, theta = %.4f rad\n', ...
        pose_name, x, y, theta);
    return;
end

end


function drawPose(x, y, theta, arrow_len, color, label, line_style)
plot(x, y, 'o', ...
    'Color', color, 'MarkerFaceColor', color, 'MarkerSize', 8);
quiver(x, y, arrow_len*cos(theta), arrow_len*sin(theta), 0, ...
    'Color', color, 'LineStyle', line_style, ...
    'LineWidth', 2, 'MaxHeadSize', 1);
text(x, y, ['  ', label], ...
    'Color', color, 'FontSize', 11, 'FontWeight', 'bold');
end


function [xmin, xmax, ymin, ymax] = determineAxisLimits(data, params)
% 优先使用全局环境范围；未初始化时由任务内容推导显示范围。

if isstruct(params) && isfield(params, 'environment') && ...
   isfield(params.environment, 'xmin') && ...
   isfield(params.environment, 'xmax') && ...
   isfield(params.environment, 'ymin') && ...
   isfield(params.environment, 'ymax')

    xmin = params.environment.xmin;
    xmax = params.environment.xmax;
    ymin = params.environment.ymin;
    ymax = params.environment.ymax;
    return;
end

all_x = [data.x0, data.xf];
all_y = [data.y0, data.yf];
for i = 1:numel(data.obs)
    all_x = [all_x, data.obs(i).x(:).']; %#ok<AGROW>
    all_y = [all_y, data.obs(i).y(:).']; %#ok<AGROW>
end

xmin = min(all_x);
xmax = max(all_x);
ymin = min(all_y);
ymax = max(all_y);

span = max([xmax - xmin, ymax - ymin, 1]);
padding = 0.05 * span;
xmin = xmin - padding;
xmax = xmax + padding;
ymin = ymin - padding;
ymax = ymax + padding;
end


function deleteIfValid(graphics_handle)
if isgraphics(graphics_handle)
    delete(graphics_handle);
end
end

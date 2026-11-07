function MakeTaskByMouse(savefile)
% MakeTaskByMouse
% 鼠标交互式绘制障碍物，并保存为 .mat 任务文件
%
% 改进内容：
% 1) 图窗范围固定，优先使用 global params 中的环境边界
% 2) 画完一个障碍物后窗口不关闭
% 3) 是否继续绘制由命令行输入 y/n 决定
% 4) 输入 n 后退出障碍物绘制，继续输入起终点信息
% 5) 绘制障碍物时，每点击一个点，立即在图上用黑点标出，并显示当前折线
% 6) 起点和终点位姿均改为鼠标两点确定：
%    第一个点为车辆后轴中心，第二个点用于确定朝向
%
% 用法:
%   MakeTaskByMouse('7.mat')
%   MakeTaskByMouse("D:\desktop\...\7.mat")

clc;
close all;
global params

%% ==================== 创建固定窗口 ====================
fig = figure('Name', 'Draw Obstacles by Mouse', ...
             'Color', 'w', ...
             'NumberTitle', 'off');
set(fig, 'outerposition', get(0, 'screensize'));

hold on; box on; grid minor; axis equal;
xlabel('x');
ylabel('y');
title({'Mouse Drawing Mode', ...
       '障碍物：左键逐点点击，按 Enter 结束当前障碍物', ...
       '起点/终点：每个位姿点击两次（后轴中心 + 朝向点）'});

% ===== 固定坐标范围：优先采用 params.environment 中的边界 =====
if isfield(params, 'environment') && ...
   isfield(params.environment, 'xmin') && ...
   isfield(params.environment, 'xmax') && ...
   isfield(params.environment, 'ymin') && ...
   isfield(params.environment, 'ymax')

    axis([params.environment.xmin, params.environment.xmax, ...
          params.environment.ymin, params.environment.ymax]);
else
    warning('未检测到 params.environment.xmin/xmax/ymin/ymax，坐标范围将由绘图自动决定。');
end

drawnow;

%% ==================== 初始化障碍物 ====================
obs = struct('x', {}, 'y', {});
obs_count = 0;
goto_next_stage = false;

%% ==================== 循环绘制障碍物 ====================
while true
    fprintf('\n==============================\n');
    fprintf('开始绘制第 %d 个障碍物...\n', obs_count + 1);
    fprintf('请在图窗中左键依次点击顶点，按 Enter 结束当前障碍物。\n');
    fprintf('每点击一个点，图上会立即显示黑点与当前边线。\n');
    fprintf('若本次不想画有效障碍物，可直接少点几个点再按 Enter。\n');

    figure(fig);

    % ---------- 逐点采集 ----------
    x = [];
    y = [];

    h_temp_pts  = gobjects(0);
    h_temp_line = gobjects(1);

    while true
        [xi, yi, button] = ginput(1);

        % Enter 结束当前障碍物
        if isempty(button)
            break;
        end

        % 只接受鼠标左键
        if button ~= 1
            fprintf('检测到非左键输入，忽略。\n');
            continue;
        end

        x(end+1,1) = xi; %#ok<AGROW>
        y(end+1,1) = yi; %#ok<AGROW>

        % 画当前点（黑点）
        h_temp_pts(end+1) = plot(xi, yi, 'ko', ...
            'MarkerFaceColor', 'k', ...
            'MarkerSize', 5, ...
            'LineWidth', 1.0); %#ok<AGROW>

        % 标注点号
        text(xi, yi, sprintf('  %d', length(x)), ...
            'Color', 'k', ...
            'FontSize', 10, ...
            'FontWeight', 'bold');

        % 更新当前折线
        if isgraphics(h_temp_line)
            delete(h_temp_line);
        end
        if length(x) >= 2
            h_temp_line = plot(x, y, 'k--', 'LineWidth', 1.0);
        end

        drawnow;
    end

    % ---------- 检查有效性 ----------
    if length(x) < 3
        fprintf('顶点数少于 3，当前障碍物无效，已忽略。\n');

        % 删除临时线，保留点也可以；这里选择保留，方便用户看到点击痕迹
        if isgraphics(h_temp_line)
            delete(h_temp_line);
        end
    else
        % ---------- 自动闭合 ----------
        if abs(x(1) - x(end)) > 1e-10 || abs(y(1) - y(end)) > 1e-10
            x = [x; x(1)];
            y = [y; y(1)];
        end

        % ---------- 删除临时折线 ----------
        if isgraphics(h_temp_line)
            delete(h_temp_line);
        end

        % ---------- 存储 ----------
        obs_count = obs_count + 1;
        obs(obs_count).x = x(:)';
        obs(obs_count).y = y(:)';

        % ---------- 正式绘制填充 ----------
        fill(obs(obs_count).x, obs(obs_count).y, [0.75 0.75 0.75], ...
             'EdgeColor', 'k', 'LineWidth', 1.5);

        % ---------- 重新覆盖顶点 ----------
        for j = 1:length(x)-1
            plot(x(j), y(j), 'ko', ...
                 'MarkerFaceColor', 'k', ...
                 'MarkerSize', 5, ...
                 'LineWidth', 1.0);
            text(x(j), y(j), sprintf('  %d', j), ...
                 'Color', 'k', ...
                 'FontSize', 10, ...
                 'FontWeight', 'bold');
        end

        % ---------- 障碍物编号 ----------
        cx = mean(x(1:end-1));
        cy = mean(y(1:end-1));
        text(cx, cy, sprintf('Obs %d', obs_count), ...
             'Color', 'b', ...
             'FontSize', 12, ...
             'FontWeight', 'bold', ...
             'HorizontalAlignment', 'center');

        fprintf('障碍物 %d 已保存，顶点数 = %d\n', obs_count, length(x)-1);
        fprintf('x = ['); fprintf(' %.4f', obs(obs_count).x); fprintf(' ]\n');
        fprintf('y = ['); fprintf(' %.4f', obs(obs_count).y); fprintf(' ]\n');

        drawnow;
    end

    % ---------- 是否继续 ----------
    while true
        str = input('是否继续绘制下一个障碍物？(y/n): ', 's');
        if strcmpi(str, 'y')
            break;
        elseif strcmpi(str, 'n')
            goto_next_stage = true;
            break;
        else
            fprintf('输入无效，请输入 y 或 n。\n');
        end
    end

    if goto_next_stage
        break;
    end
end

%% ==================== 检查是否有有效障碍物 ====================
if obs_count == 0
    warning('没有绘制任何有效障碍物。将仍然继续输入起终点并保存。');
end

%% ==================== 输入起点和终点（鼠标两点定姿态） ====================
fprintf('\n==============================\n');
fprintf('接下来请用鼠标输入起点位姿和终点位姿。\n');
fprintf('每个位姿点击两次：\n');
fprintf('  第一次：车辆后轴中心位置\n');
fprintf('  第二次：朝向参考点\n');

figure(fig);

% 箭头长度：参考环境尺寸自动设定
if isfield(params, 'environment') && ...
   isfield(params.environment, 'xmin') && ...
   isfield(params.environment, 'xmax') && ...
   isfield(params.environment, 'ymin') && ...
   isfield(params.environment, 'ymax')

    env_dx = params.environment.xmax - params.environment.xmin;
    env_dy = params.environment.ymax - params.environment.ymin;
    L = 0.05 * max(env_dx, env_dy);
else
    L = 1.5;
end

% ---------- 起点 ----------
fprintf('\n请选择起点位姿：\n');
fprintf('第1次点击：起点后轴中心\n');
fprintf('第2次点击：起点朝向参考点\n');

[x0, y0, theta0] = getPoseByTwoClicks(fig, 'Start');

% 绘制起点
plot(x0, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
quiver(x0, y0, L*cos(theta0), L*sin(theta0), 0, ...
       'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 1);
text(x0, y0, '  Start', ...
     'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

drawnow;

% ---------- 终点 ----------
fprintf('\n请选择终点位姿：\n');
fprintf('第1次点击：终点后轴中心\n');
fprintf('第2次点击：终点朝向参考点\n');

[xf, yf, thetaf] = getPoseByTwoClicks(fig, 'Goal');

% 绘制终点
plot(xf, yf, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
quiver(xf, yf, L*cos(thetaf), L*sin(thetaf), 0, ...
       'Color', 'm', 'LineWidth', 2, 'MaxHeadSize', 1);
text(xf, yf, '  Goal', ...
     'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');

drawnow;

%% ==================== 保存 ====================
save(savefile, 'obs', 'x0', 'y0', 'theta0', 'xf', 'yf', 'thetaf');

fprintf('\n任务文件已保存到:\n%s\n', savefile);
fprintf('包含变量: obs, x0, y0, theta0, xf, yf, thetaf\n');

end

%% ============================================================
function [x, y, theta] = getPoseByTwoClicks(fig, pose_name)
% 用鼠标点击两次确定一个位姿
% 第一次：位置
% 第二次：朝向参考点

figure(fig);

while true
    [x1, y1, b1] = ginput(1);
    if isempty(b1) || b1 ~= 1
        fprintf('%s 第一个点无效，请重新点击。\n', pose_name);
        continue;
    end

    % 先画中心点
    plot(x1, y1, 'ks', 'MarkerFaceColor', 'y', 'MarkerSize', 6, 'LineWidth', 1.0);
    text(x1, y1, sprintf('  %s-Center', pose_name), ...
        'Color', 'k', 'FontSize', 10, 'FontWeight', 'bold');
    drawnow;

    [x2, y2, b2] = ginput(1);
    if isempty(b2) || b2 ~= 1
        fprintf('%s 第二个点无效，请重新选择该位姿。\n', pose_name);
        continue;
    end

    dx = x2 - x1;
    dy = y2 - y1;

    if hypot(dx, dy) < 1e-10
        fprintf('%s 两个点过近，无法确定方向，请重新选择。\n', pose_name);
        continue;
    end

    % 画方向参考线和参考点
    plot([x1, x2], [y1, y2], 'k-.', 'LineWidth', 1.2);
    plot(x2, y2, 'k.', 'MarkerSize', 15);

    x = x1;
    y = y1;
    theta = atan2(dy, dx);

    fprintf('%s 位姿已确定：x = %.4f, y = %.4f, theta = %.4f rad\n', ...
        pose_name, x, y, theta);
    break;
end

end
function EditTaskObstacles(matfile, savefile)
global params
% EditTaskObstacles 编辑已有环境中的障碍物
%
% 用法：
%   EditTaskObstacles('7.mat')
%   EditTaskObstacles('7.mat', '8.mat')
%
% 功能：
%   1. 加载已有环境
%   2. 可删除指定障碍物
%   3. 可鼠标新增障碍物
%   4. 保存为原文件或新文件
%
% 改进：
%   1. 起点/终点显示位姿箭头
%   2. 新增障碍物时，实时显示点击顶点（黑点+虚线）

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

if nargin < 2
    savefile = matfile;
end

clc;
close all;

S = load(matfile);

required_vars = {'obs','x0','y0','theta0','xf','yf','thetaf'};
for k = 1:length(required_vars)
    if ~isfield(S, required_vars{k})
        error('文件中缺少变量: %s', required_vars{k});
    end
end

obs    = S.obs;
x0     = S.x0;
y0     = S.y0;
theta0 = S.theta0;
xf     = S.xf;
yf     = S.yf;
thetaf = S.thetaf;

% ===== 位姿箭头长度（随地图大小自适应）=====
arrow_len = 0.06 * min(env_dx, env_dy);

while true
    %% ===== 开始绘图 =====
    figure('Name', 'Edit Task Obstacles', 'Color', 'w');
    set(gcf, 'outerposition', get(0,'screensize'));
    hold on; box on; grid minor; axis equal;

    axis([xmin, xmax, ymin, ymax]);
    xlabel('x / m');
    ylabel('y / m');
    title('Current Environment');

    % ===== 绘制障碍物 =====
    for i = 1:length(obs)
        fill(obs(i).x, obs(i).y, [0.75 0.75 0.75], ...
            'EdgeColor', 'k', 'LineWidth', 1.5);

        % 若最后一个点与第一个点重复，则计算中心时去掉最后一个点
        if length(obs(i).x) >= 2 && ...
           abs(obs(i).x(1) - obs(i).x(end)) < 1e-10 && ...
           abs(obs(i).y(1) - obs(i).y(end)) < 1e-10
            cx = mean(obs(i).x(1:end-1));
            cy = mean(obs(i).y(1:end-1));
        else
            cx = mean(obs(i).x);
            cy = mean(obs(i).y);
        end

        text(cx, cy, sprintf('Obs %d', i), ...
            'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center');
    end

    % ===== 绘制起点位姿 =====
    plot(x0, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
    quiver(x0, y0, arrow_len*cos(theta0), arrow_len*sin(theta0), 0, ...
        'Color', [0 0.6 0], 'LineWidth', 2, 'MaxHeadSize', 1.2);
    text(x0, y0, '  Start', 'Color', [0 0.5 0], ...
        'FontSize', 12, 'FontWeight', 'bold');

    % ===== 绘制终点位姿 =====
    plot(xf, yf, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
    quiver(xf, yf, arrow_len*cos(thetaf), arrow_len*sin(thetaf), 0, ...
        'Color', [0.7 0 0.7], 'LineWidth', 2, 'MaxHeadSize', 1.2);
    text(xf, yf, '  Goal', 'Color', [0.6 0 0.6], ...
        'FontSize', 12, 'FontWeight', 'bold');

    drawnow;

    fprintf('\n当前共有 %d 个障碍物。\n', length(obs));
    fprintf('1 - 删除障碍物\n');
    fprintf('2 - 新增障碍物\n');
    fprintf('3 - 结束并保存\n');

    op = input('请选择操作: ');

    if op == 1
        idx = input('请输入要删除的障碍物编号: ');
        if idx >= 1 && idx <= length(obs)
            obs(idx) = [];
            fprintf('已删除障碍物 %d\n', idx);
        else
            fprintf('编号无效。\n');
        end
        close gcf;

    elseif op == 2
        % ===== 新增障碍物：实时点选显示 =====
        cla; hold on; box on; grid minor; axis equal;
        axis([xmin, xmax, ymin, ymax]);
        xlabel('x / m');
        ylabel('y / m');
        title({'Add New Obstacle', ...
               '左键逐点选择顶点；按 Enter 结束；至少 3 个点'}, ...
               'FontWeight', 'bold');

        % 先重新画原环境
        for i = 1:length(obs)
            fill(obs(i).x, obs(i).y, [0.75 0.75 0.75], ...
                'EdgeColor', 'k', 'LineWidth', 1.5);

            if length(obs(i).x) >= 2 && ...
               abs(obs(i).x(1) - obs(i).x(end)) < 1e-10 && ...
               abs(obs(i).y(1) - obs(i).y(end)) < 1e-10
                cx = mean(obs(i).x(1:end-1));
                cy = mean(obs(i).y(1:end-1));
            else
                cx = mean(obs(i).x);
                cy = mean(obs(i).y);
            end

            text(cx, cy, sprintf('Obs %d', i), ...
                'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center');
        end

        % 起终点位姿也保留显示
        plot(x0, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
        quiver(x0, y0, arrow_len*cos(theta0), arrow_len*sin(theta0), 0, ...
            'Color', [0 0.6 0], 'LineWidth', 2, 'MaxHeadSize', 1.2);
        text(x0, y0, '  Start', 'Color', [0 0.5 0], ...
            'FontSize', 12, 'FontWeight', 'bold');

        plot(xf, yf, 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8);
        quiver(xf, yf, arrow_len*cos(thetaf), arrow_len*sin(thetaf), 0, ...
            'Color', [0.7 0 0.7], 'LineWidth', 2, 'MaxHeadSize', 1.2);
        text(xf, yf, '  Goal', 'Color', [0.6 0 0.6], ...
            'FontSize', 12, 'FontWeight', 'bold');

        drawnow;

        fprintf('请在图中左键点击新障碍物顶点，按 Enter 结束。\n');

        x = [];
        y = [];

        while true
            [xi, yi, button] = ginput(1);

            % 按 Enter 结束时，ginput 返回空
            if isempty(xi) || isempty(yi)
                break;
            end

            % 只处理鼠标左键/普通点击
            if isempty(button)
                break;
            end

            x(end+1,1) = xi;
            y(end+1,1) = yi;

            % 画当前顶点黑点
            plot(xi, yi, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

            % 若点数 >= 2，画顶点间虚线
            if length(x) >= 2
                plot(x(end-1:end), y(end-1:end), 'k--', 'LineWidth', 1.2);
            end

            % 点编号
            text(xi, yi, sprintf('  %d', length(x)), ...
                'Color', 'k', 'FontSize', 10, 'FontWeight', 'bold');

            title({sprintf('Add New Obstacle  |  当前已选 %d 个点', length(x)), ...
                   '左键逐点选择顶点；按 Enter 结束；至少 3 个点'}, ...
                   'FontWeight', 'bold');

            drawnow;
        end

        if length(x) < 3
            fprintf('顶点不足 3 个，新增失败。\n');
        else
            % 自动闭合
            if abs(x(1)-x(end)) > 1e-10 || abs(y(1)-y(end)) > 1e-10
                x = [x; x(1)];
                y = [y; y(1)];
            end

            % 补一条闭合虚线，便于看闭合效果
            plot([x(end-1), x(end)], [y(end-1), y(end)], 'k--', 'LineWidth', 1.2);

            % 用浅灰填充预览
            fill(x, y, [0.85 0.85 0.85], ...
                'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 0.5);
            drawnow;

            obs(end+1).x = x(:)';
            obs(end).y   = y(:)';

            fprintf('已新增障碍物 %d\n', length(obs));
        end

        pause(0.5);
        close gcf;

    elseif op == 3
        close gcf;
        break;

    else
        fprintf('无效操作。\n');
        close gcf;
    end
end

save(savefile, 'obs', 'x0', 'y0', 'theta0', 'xf', 'yf', 'thetaf');
fprintf('\n已保存到: %s\n', savefile);

end
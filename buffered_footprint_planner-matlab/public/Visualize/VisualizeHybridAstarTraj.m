function VisualizeHybridAstarTraj()
% 只画：障碍物 + 起终点位姿 + Hybrid A* 最终路径 + 采样点
% 前进点：蓝色
% 倒车点：红色
% 采样点：黑色小圆圈

global params

figure('Name','Hybrid A* Path','Color','w');
set(gcf, 'outerposition', get(0,'screensize'));
hold on; box on; grid minor; axis equal;

% 坐标范围
if isfield(params, 'environment') && isfield(params.environment, 'xmin')
    axis([params.environment.xmin, params.environment.xmax, ...
          params.environment.ymin, params.environment.ymax]);
end

% ================= 障碍物 =================
h_obs = gobjects(params.environment.num_obs,1);
for ii = 1 : params.environment.num_obs
    h_obs(ii) = fill(params.environment.obs(ii).x, ...
                     params.environment.obs(ii).y, ...
                     [0.75 0.75 0.75], 'EdgeColor', 'none');
end

% ================= 起终点 =================
x0 = params.task.x0;  y0 = params.task.y0;  th0 = params.task.theta0;
xf = params.task.xf;  yf = params.task.yf;  thf = params.task.thetaf;

h_start = plot(x0, y0, 'go', 'MarkerSize', 10, 'LineWidth', 2);
h_goal  = plot(xf, yf, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

L = 1.0; % 箭头长度
if exist('Arrow', 'file') == 2
    Arrow([x0, y0], [x0 + L*cos(th0), y0 + L*sin(th0)], ...
          'Length', 10, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 1);
    Arrow([xf, yf], [xf + L*cos(thf), yf + L*sin(thf)], ...
          'Length', 10, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 1);
else
    quiver(x0, y0, L*cos(th0), L*sin(th0), 0, 'g', ...
           'LineWidth', 2, 'MaxHeadSize', 2);
    quiver(xf, yf, L*cos(thf), L*sin(thf), 0, 'r', ...
           'LineWidth', 2, 'MaxHeadSize', 2);
end

% ================= Hybrid A* 路径 =================
if ~isfield(params, 'ha') || ~isfield(params.ha, 'x') || isempty(params.ha.x)
    error('params.ha.x/y 为空：请先运行 SearchTrajViaHybridAstar()');
end

x = params.ha.x;
y = params.ha.y;

% 如果没有速度信息，默认全部前进
if isfield(params.ha, 'v') && ~isempty(params.ha.v)
    v = params.ha.v;
else
    warning('未检测到 params.ha.v，默认全部为前进');
    v = ones(size(x));
end

% ================= 先画整条路径连线 =================
h_path = plot(x, y, 'k--', 'LineWidth', 1.0);

% ================= 前进 / 倒车 分离 =================
forward_idx  = v > 0;
backward_idx = v < 0;
zero_idx     = v == 0;

h_forward  = plot(x(forward_idx),  y(forward_idx),  'b.', 'MarkerSize', 12);
h_backward = plot(x(backward_idx), y(backward_idx), 'r.', 'MarkerSize', 12);

% 若存在 v=0 的点，也画出来
if any(zero_idx)
    h_zero = plot(x(zero_idx), y(zero_idx), 'mo', ...
                  'MarkerSize', 6, 'LineWidth', 1.5);
else
    h_zero = gobjects(1);
end

% ================= 标出采样点位置 =================
h_sample = plot(x, y, 'ko', ...
                'MarkerSize', 4, ...
                'LineWidth', 1.0, ...
                'MarkerFaceColor', 'y');

% ================= 可选：标注采样点序号 =================
show_index = true;     % 想显示编号就 true，不想显示就 false
index_step = 5;        % 每隔几个点标一个序号，避免太密

if show_index
    for k = 1:index_step:length(x)
        text(x(k), y(k), sprintf('%d', k), ...
            'FontSize', 8, ...
            'Color', [0.1 0.1 0.1], ...
            'FontWeight', 'bold', ...
            'VerticalAlignment', 'bottom', ...
            'HorizontalAlignment', 'left');
    end
end

% ================= 若有换向点，也顺手圈出来 =================
chg = find(v(2:end) .* v(1:end-1) < 0) + 1;
if ~isempty(chg)
    plot(x(chg), y(chg), 'cs', ...
         'MarkerSize', 8, ...
         'LineWidth', 1.5);
end

% ================= 坐标与图例 =================
set(gca,'FontWeight','bold','FontSize',18,'FontName','Arial');
xlabel('x / m');
ylabel('y / m');
title(sprintf('Task %d: Hybrid A* Initial Path with Sample Points', params.task_id));

legend_list = [h_obs(1), h_start, h_goal, h_path, h_forward, h_backward, h_sample];
legend_name = {'Obstacle','Start','Goal','Path','Forward','Backward','Sample Point'};

if any(zero_idx)
    legend_list = [legend_list, h_zero];
    legend_name = [legend_name, {'v = 0'}];
end

legend(legend_list, legend_name, 'Location', 'best');

drawnow;

end
function visualize_compare()
% 绘制混合A*轨迹 + 优化后的轨迹对比
% 蓝色：混合A*轨迹
% 红色：优化轨迹

global params

figure;
set(gcf, 'outerposition', get(0,'screensize'));
hold on; box on; grid minor; axis equal;

% 坐标范围
if isfield(params, 'environment') && isfield(params.environment, 'xmin')
    axis([params.environment.xmin, params.environment.xmax, ...
          params.environment.ymin, params.environment.ymax]);
end

% ================= 障碍物 =================
if isfield(params.environment,'obs') && ~isempty(params.environment.obs)
    for ii = 1:length(params.environment.obs)
        fill(params.environment.obs(ii).x, params.environment.obs(ii).y, ...
             [0.75 0.75 0.75], 'EdgeColor','none');
    end
end

% ================= 起终点 =================
x0 = params.task.x0; y0 = params.task.y0; th0 = params.task.theta0;
xf = params.task.xf; yf = params.task.yf; thf = params.task.thetaf;

% 起点圆点
plot(x0, y0, 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0 1 0], 'MarkerEdgeColor', [0 0.6 0]);  % 绿色圆点
% 终点圆点
plot(xf, yf, 'o', 'MarkerSize', 12, 'MarkerFaceColor', [0.65 0.16 0], 'MarkerEdgeColor', [0.5 0.2 0]);  % 棕色圆点

% 箭头表示姿态
L = 1.0;  % 长箭头
LineWidthArrow = 2; 
MaxHeadSizeArrow = 4;

% 起点虚线绿色箭头
quiver(x0, y0, L*cos(th0), L*sin(th0), 0, 'Color', [0 1 0], ...
       'LineStyle', '-', 'LineWidth', LineWidthArrow, 'MaxHeadSize', MaxHeadSizeArrow);

% 终点虚线棕色箭头
quiver(xf, yf, L*cos(thf), L*sin(thf), 0, 'Color', [1 0 1], ...
       'LineStyle', '-', 'LineWidth', LineWidthArrow, 'MaxHeadSize', MaxHeadSizeArrow);

% ================= 混合A*轨迹 =================
if ~isfield(params,'ha') || isempty(params.ha.x)
    error('params.ha.x/y 为空：请先运行 SearchTrajViaHybridAstar()');
end
x_ha = params.ha.x;
y_ha = params.ha.y;

h_ha = plot(x_ha, y_ha, '-', 'Color', [0.15 0.40 0.95], 'LineWidth', 2); % 学术蓝

% ================= 优化后的轨迹 =================
if ~isfield(params,'ef') || isempty(params.ef.x)
    warning('params.ef.x/y 为空，无法绘制优化轨迹');
else
    x_opt = params.ef.x;
    y_opt = params.ef.y;
    h_opt = plot(x_opt, y_opt, '-', 'Color', [0.90 0.15 0.15], 'LineWidth', 2); % 学术红
end

% ================= 起终点 =================
h_start = plot(x0, y0, 'go', 'MarkerSize', 10, 'LineWidth', 2);
h_goal  = plot(xf, yf, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% % ================= 图例 =================
% legend_handles = [h_ha, h_opt, h_start, h_goal];
% legend_labels = {'混合A*路径', '优化轨迹', '起点', '终点'};
% legend(legend_handles, legend_labels, 'Location', 'best');

drawnow;

end
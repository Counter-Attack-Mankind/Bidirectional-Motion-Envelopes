function VisualizeHybridAstarPath()
% 只画：障碍物 + 起终点位姿 + Hybrid A* 最终路径
% 前进点：蓝色
% 倒车点：红色

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
quiver(x0, y0, L*cos(th0), L*sin(th0), 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 2);
quiver(xf, yf, L*cos(thf), L*sin(thf), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
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

% ================= 前进 / 倒车 分离 =================
forward_idx  = v > 0;
backward_idx = v < 0;

h_forward = plot(x(forward_idx),  y(forward_idx),  'b.', 'MarkerSize', 8);
h_backward = plot(x(backward_idx), y(backward_idx), 'r.', 'MarkerSize', 8);

% ================= 坐标与图例 =================
set(gca,'FontWeight','bold','FontSize',18,'FontName','Arial');
xlabel('x / m'); ylabel('y / m');
% title(sprintf('Task %d: Hybrid A* Initial Path', params.task_id));
% 
% legend([h_obs(1), h_start, h_goal, h_forward, h_backward], ...
% {'Obstacle','Start','Goal','Forward','Backward'}, ...
% 'Location','best');

drawnow;

end

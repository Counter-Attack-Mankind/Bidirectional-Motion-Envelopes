function VisualizeEmbodimentFilteredTraj(x, y)
% 画：障碍物 + 起终点 + 具身足迹筛选后的关键点轨迹（ConvertPathToTraj 输出）
global params

figure(3);
set(gcf, 'outerposition', get(0,'screensize'));
hold on; box on; grid minor; axis equal;

% 坐标范围
if isfield(params, 'environment') && isfield(params.environment, 'xmin')
    axis([params.environment.xmin, params.environment.xmax, ...
          params.environment.ymin, params.environment.ymax]);
end

% 障碍物
h_obs = gobjects(params.environment.num_obs,1);
for ii = 1 : params.environment.num_obs
    h_obs(ii) = fill(params.environment.obs(ii).x, ...
                     params.environment.obs(ii).y, ...
                     [0.75 0.75 0.75], 'EdgeColor', 'none');
end


% 起终点位姿（点 + 朝向箭头）
x0 = params.task.x0;  y0 = params.task.y0;  th0 = params.task.theta0;
xf = params.task.xf;  yf = params.task.yf;  thf = params.task.thetaf;

h_start = plot(x0, y0, 'go', 'MarkerSize', 10, 'LineWidth', 2);
h_goal  = plot(xf, yf, 'ro', 'MarkerSize', 10, 'LineWidth', 2);

L = 1.0;
if exist('Arrow', 'file') == 2
    Arrow([x0, y0], [x0 + L*cos(th0), y0 + L*sin(th0)], ...
          'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2);
    Arrow([xf, yf], [xf + L*cos(thf), yf + L*sin(thf)], ...
          'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2);
else
    quiver(x0, y0, L*cos(th0), L*sin(th0), 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 2);
    quiver(xf, yf, L*cos(thf), L*sin(thf), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
end

% 画具身筛选后的关键点（点 + 连线）
h_path = plot(x, y, 'm-', 'LineWidth', 1.5);
h_point = plot(x, y, 'mo', 'MarkerSize', 7, 'MarkerFaceColor', 'm');

% 显示点数
text(0.02, 0.98, sprintf('Key points: %d', length(x)), ...
     'Units', 'normalized', 'FontSize', 16, 'FontWeight', 'bold', ...
     'VerticalAlignment', 'top');

set(gca,'FontWeight','bold','FontSize',18,'FontName','Arial');
xlabel('x / m'); ylabel('y / m');
% title(sprintf('Task %d: Embodiment-filtered Points', params.task_id));
% legend([h_obs(1), h_start, h_goal, h_path, h_point], ...
%        {'Obstacle','Start','Goal','Initial guess path', 'Embodiment-filtered  Points'}, ...
%        'Location','best');
drawnow;
end

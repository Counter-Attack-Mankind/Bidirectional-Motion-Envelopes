function VisualizeMapAndPose()
% 只可视化：障碍物 + 起点/终点位姿（含朝向）
global params

figure; 
set(gcf, 'outerposition', get(0,'screensize'));
hold on; box on; grid minor; axis equal;

% 坐标范围
if isfield(params, 'environment') && isfield(params.environment, 'xmin')
    axis([params.environment.xmin, params.environment.xmax, ...
          params.environment.ymin, params.environment.ymax]);
end

% 画障碍物（多边形填充）
if isfield(params, 'environment') && isfield(params.environment, 'num_obs')
    for ii = 1 : params.environment.num_obs
        fill(params.environment.obs(ii).x, params.environment.obs(ii).y, ...
             [0.75, 0.75, 0.75], 'EdgeColor', 'none');
    end
end

% 坐标轴字体
set(gca,'FontWeight','bold','FontSize',18,'FontName','Arial');
xlabel('x / m'); ylabel('y / m');

% 起点 / 终点位姿（箭头 + 点）
x0 = params.task.x0;  y0 = params.task.y0;  th0 = params.task.theta0;
xf = params.task.xf;  yf = params.task.yf;  thf = params.task.thetaf;


% 朝向箭头
L = 1.0; % 箭头长度，可按地图尺度改
if exist('Arrow', 'file') == 2
    Arrow([x0, y0], [x0 + L*cos(th0), y0 + L*sin(th0)], ...
          'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2);
    Arrow([xf, yf], [xf + L*cos(thf), yf + L*sin(thf)], ...
          'Length', 16, 'BaseAngle', 90, 'TipAngle', 16, 'Width', 2);

legend({'Start','Goal'}, 'Location', 'best');
title(sprintf('Task %d: Map + Start/Goal Pose', params.task_id));

drawnow;
end

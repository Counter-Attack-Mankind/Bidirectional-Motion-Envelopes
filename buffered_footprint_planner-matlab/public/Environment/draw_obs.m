clc; clear; close all;

thisFile = mfilename('fullpath');
if isempty(thisFile)
    thisDir = pwd;
else
    thisDir = fileparts(thisFile);
end
projectRoot = fileparts(thisDir);
addpath(fullfile(projectRoot, 'Common'));

global params   
InitializeParams();
LF  = params.vehicle.LF;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% ========= 运动参数（由 v, kappa, t 决定） =========
v     = -0.2;     % m/s，可正可负
kappa = 0.4;      % 1/m，可正可负
tspan = 8;      % s

distance = v * tspan;         % 弧长
theta    = kappa * distance;  % 总转角

%% ========= t 时刻后轴中心位姿 =========
x0 = 0;
y0 = 0;
yaw0 = pi/2;

%% ========= 计算 t1 时刻后轴中心位姿 =========
if abs(kappa) < 1e-8
    x1 = x0 + distance * cos(yaw0);
    y1 = y0 + distance * sin(yaw0);
    yaw1 = yaw0;
else
    x1 = x0 + (sin(yaw0 + theta) - sin(yaw0)) / kappa;
    y1 = y0 - (cos(yaw0 + theta) - cos(yaw0)) / kappa;
    yaw1 = yaw0 + theta;
end

%% ========= 生成扫掠区域 =========
N = 120;

if abs(kappa) < 1e-8
    xs   = linspace(x0, x1, N);
    ys   = linspace(y0, y1, N);
    yaws = linspace(yaw0, yaw1, N);
else
    ss   = linspace(0, distance, N);
    yaws = yaw0 + kappa * ss;

    xs = x0 + (sin(yaws) - sin(yaw0)) / kappa;
    ys = y0 - (cos(yaws) - cos(yaw0)) / kappa;
end

allPts = [];

for i = 1:N
    V = getVehicleBox(xs(i), ys(i), yaws(i), LF, lr, hlb);  % 4x2
    allPts = [allPts; V]; %#ok<AGROW>
end

% 求扫掠外边界
shp = alphaShape(allPts(:,1), allPts(:,2), Inf);
[bf, P] = boundaryFacets(shp);

%% ========= 开始绘图 =========
figure('Color','w', 'Position', [100 100 700 700]);
hold on; box on;
axis equal;
daspect([1 1 1]);

% 固定地图显示范围（你自己按需要改）
xlim([-5 5]);
ylim([-5 5]);
set(gca, 'FontSize', 12);

t1Color = [0.85 0.45 0.10];   % 橙色

% -------------------------------------------------
% 1) 画从 k 到 k+1 的连续扫掠区域
% -------------------------------------------------
h_swept = plot(shp, ...
    'FaceColor', [0.78 0.86 0.94], ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.85);

% -------------------------------------------------
% 2) 第 k 个配置点时刻的车身足迹
% -------------------------------------------------
V0 = getVehicleBox(x0, y0, yaw0, LF, lr, hlb);
h_k = patch(V0(:,1), V0(:,2), [0.85 0.9 0.9], ...
    'EdgeColor', [0 0.75 0.6], ...
    'LineWidth', 2);

% -------------------------------------------------
% 3) 第 k+1 个配置点时刻的车身足迹
% 保持为 patch，这样图例里显示的是"虚线矩形"
% -------------------------------------------------
V1 = getVehicleBox(x1, y1, yaw1, LF, lr, hlb);
h_k1 = patch(V1(:,1), V1(:,2), 'w', ...
    'FaceColor', 'none', ...
    'EdgeColor', t1Color, ...
    'LineStyle', '--', ...
    'LineWidth', 2);

% -------------------------------------------------
% 4) 前轮转角
% -------------------------------------------------
phy0 = atan(params.vehicle.lw * kappa);
phy1 = phy0;

% -------------------------------------------------
% 5) 在 k 时刻画轮子（不进图例）
% -------------------------------------------------
DrawWheelsStyled(x0, y0, yaw0, phy0, 'solid', []);

% -------------------------------------------------
% 6) 在 k+1 时刻画轮子（不进图例）
% -------------------------------------------------
DrawWheelsStyled(x1, y1, yaw1, phy1, 'dashed', t1Color);

% -------------------------------------------------
% 7) 后轴轨迹（不进图例）
% -------------------------------------------------
plot(xs, ys, 'k--', 'LineWidth', 1.2, ...
    'HandleVisibility', 'off');

% -------------------------------------------------
% 8) 标出 k 和 k+1 的后轴中心（不进图例）
% -------------------------------------------------
plot(x0, y0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, ...
    'HandleVisibility', 'off');
plot(x1, y1, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, ...
    'HandleVisibility', 'off');

text(x0+0.08, y0-0.18, 'O_k', 'FontSize', 16, ...
    'FontName', 'Times New Roman', 'FontAngle', 'italic', ...
    'HandleVisibility', 'off');
text(x1+0.08, y1-0.18, 'O_{k+1}', 'FontSize', 16, ...
    'FontName', 'Times New Roman', 'FontAngle', 'italic', ...
    'HandleVisibility', 'off');

% -------------------------------------------------
% 9) 画坐标轴（不进图例）
% -------------------------------------------------
L = 4;

plot([-L L], [0 0], 'k', 'LineWidth', 2, ...
    'HandleVisibility', 'off');
plot([0 0], [-L L], 'k', 'LineWidth', 2, ...
    'HandleVisibility', 'off');

quiver(L-0.4, 0, 0.4, 0, 0, 'k', ...
    'LineWidth', 2, 'MaxHeadSize', 0.8, ...
    'HandleVisibility', 'off');
quiver(0, L-0.4, 0, 0.4, 0, 'k', ...
    'LineWidth', 2, 'MaxHeadSize', 0.8, ...
    'HandleVisibility', 'off');

text(L+0.1, -0.2, 'X', 'FontSize', 18, ...
    'FontName', 'Times New Roman', 'FontAngle', 'italic', ...
    'HandleVisibility', 'off');

text(0.1, L+0.1, 'Y', 'FontSize', 18, ...
    'FontName', 'Times New Roman', 'FontAngle', 'italic', ...
    'HandleVisibility', 'off');

% -------------------------------------------------
% 10) 尖锐三角形障碍物
% 放到最后画，使其位于顶层
% -------------------------------------------------
obstacle_pts = [1.45  2.00;
                4.00  1.55;
                3.55  3.68];

h_obs = patch(obstacle_pts(:,1), obstacle_pts(:,2), [0.72 0.72 0.72], ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.4);

uistack(h_obs, 'top');   % 明确置顶

% -------------------------------------------------
% 11) 标题和坐标轴
% -------------------------------------------------
title('Vehicle footprints and swept region from k to k+1');
xlabel('x / m');
ylabel('y / m');

%加看不见的矩形
% 用坐标范围外的闭合虚线矩形，作为图例代理对象
x_rect = [1e6, 1e6+1, 1e6+1, 1e6, 1e6];
y_rect = [1e6, 1e6,   1e6+1, 1e6+1, 1e6];

% 用填充矩形作为 legend 代理
h_k1_legend = patch(nan, nan, [1 1 1], ...
    'EdgeColor', t1Color, ...
    'LineStyle', '--', ...
    'LineWidth', 2);
% -------------------------------------------------
% 12) 图例
% -------------------------------------------------
legend([h_k, h_k1_legend, h_swept, h_obs], ...
    {'第k个配置点时刻的车身足迹', ...
     '第k+1个配置点时刻的车身足迹', ...
     '从第k到第k+1的配置点的车身连续扫掠区域', ...
     '环境中尖锐障碍物'}, ...
    'Location','northeast', ...
    'FontSize',10, ...
    'Box','on');

% Export an editable vector graphic for Visio. The same file is overwritten
% every time this script runs.
set(gcf, 'Renderer', 'painters');
drawnow;
print(gcf, '-dmeta', '-painters', fullfile(pwd, 'vehicle.emf'));
%% ========= 局部函数 =========
function V = getVehicleBox(x, y, yaw, LF, lr, hlb)
    % 车辆四个角点（以后轴中心为参考原点）
    % 左前、右前、右后、左后
    local = [ LF,  hlb;
              LF, -hlb;
             -lr, -hlb;
             -lr,  hlb ];

    R = [cos(yaw), -sin(yaw);
         sin(yaw),  cos(yaw)];

    V = (R * local')';
    V(:,1) = V(:,1) + x;
    V(:,2) = V(:,2) + y;
end

function drawHeading(x, y, yaw, L, colorv)
    quiver(x, y, L*cos(yaw), L*sin(yaw), 0, ...
        'Color', colorv, 'LineWidth',1.8, 'MaxHeadSize',0.25);
end

function DrawWheelsStyled(x, y, theta, phy, mode, customColor)
% mode = 'solid'  : t时刻，实心轮胎 + 实线车轴
% mode = 'dashed' : t+1时刻，虚线轮胎 + 虚线车轴

global params

lw  = params.vehicle.lw;
hlb = params.vehicle.hlb;

wheel_len = 0.38;
wheel_w   = 0.14;

AA = [x - sin(theta)*hlb + lw*cos(theta), y + cos(theta)*hlb + lw*sin(theta)];
BB = [x + sin(theta)*hlb + lw*cos(theta), y - cos(theta)*hlb + lw*sin(theta)];
CC = [x + sin(theta)*hlb,                 y - cos(theta)*hlb];
DD = [x - sin(theta)*hlb,                 y + cos(theta)*hlb];

switch lower(mode)
    case 'solid'
        wheelFaceColor = [0.25 0.25 0.25];
        wheelEdgeColor = [0.15 0.15 0.15];
        wheelLineStyle = '-';
        axleColor      = 'k';
        axleLineStyle  = '-';
        wheelLineWidth = 1.0;
        axleLineWidth  = 1.5;
        fillWheel      = true;

    case 'dashed'
        if isempty(customColor)
            customColor = [0.85 0.45 0.10];
        end
        wheelFaceColor = 'none';
        wheelEdgeColor = customColor;
        wheelLineStyle = '--';
        axleColor      = customColor;
        axleLineStyle  = '--';
        wheelLineWidth = 1.4;
        axleLineWidth  = 1.4;
        fillWheel      = false;

    otherwise
        error('mode must be ''solid'' or ''dashed''.');
end

drawOneWheel(CC, theta,       wheel_len, wheel_w, wheelFaceColor, wheelEdgeColor, wheelLineStyle, wheelLineWidth, fillWheel);
drawOneWheel(DD, theta,       wheel_len, wheel_w, wheelFaceColor, wheelEdgeColor, wheelLineStyle, wheelLineWidth, fillWheel);
drawOneWheel(AA, theta + phy, wheel_len, wheel_w, wheelFaceColor, wheelEdgeColor, wheelLineStyle, wheelLineWidth, fillWheel);
drawOneWheel(BB, theta + phy, wheel_len, wheel_w, wheelFaceColor, wheelEdgeColor, wheelLineStyle, wheelLineWidth, fillWheel);

plot([AA(1), BB(1)], [AA(2), BB(2)], ...
    'Color', axleColor, 'LineStyle', axleLineStyle, 'LineWidth', axleLineWidth);
plot([CC(1), DD(1)], [CC(2), DD(2)], ...
    'Color', axleColor, 'LineStyle', axleLineStyle, 'LineWidth', axleLineWidth);
end

function drawOneWheel(center, ang, wheel_len, wheel_w, faceColor, edgeColor, lineStyle, lineWidth, fillWheel)
ct = cos(ang);
st = sin(ang);

% 轮胎矩形四角
p1 = center + [ wheel_len*ct - wheel_w*st,  wheel_len*st + wheel_w*ct];
p2 = center + [ wheel_len*ct + wheel_w*st,  wheel_len*st - wheel_w*ct];
p3 = center + [-wheel_len*ct + wheel_w*st, -wheel_len*st - wheel_w*ct];
p4 = center + [-wheel_len*ct - wheel_w*st, -wheel_len*st + wheel_w*ct];

P = [p1; p2; p3; p4];

if fillWheel
    patch(P(:,1), P(:,2), [0.3 0.3 0.3], ...
        'FaceColor', faceColor, ...
        'EdgeColor', edgeColor, ...
        'LineStyle', lineStyle, ...
        'LineWidth', lineWidth);
else
    patch(P(:,1), P(:,2), 'w', ...
        'FaceColor', 'none', ...
        'EdgeColor', edgeColor, ...
        'LineStyle', lineStyle, ...
        'LineWidth', lineWidth);
end
end

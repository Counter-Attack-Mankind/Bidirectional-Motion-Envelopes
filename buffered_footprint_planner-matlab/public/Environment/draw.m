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

%% Motion parameters: determined by v, kappa, and tspan
v     = -0.05;     % m/s
kappa = -0.4;       % 1/m
tspan = 10;        % s

distance = v * tspan;
theta    = kappa * distance;

%% Pose of rear-axle center at t
x0 = 0;
y0 = 0;
yaw0 = pi/2;

%% Pose of rear-axle center at t1
if abs(kappa) < 1e-8
    x1 = x0 + distance * cos(yaw0);
    y1 = y0 + distance * sin(yaw0);
    yaw1 = yaw0;
else
    x1 = x0 + (sin(yaw0 + theta) - sin(yaw0)) / kappa;
    y1 = y0 - (cos(yaw0 + theta) - cos(yaw0)) / kappa;
    yaw1 = yaw0 + theta;
end

%% Generate swept area
N = 240;

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

% Do not use alphaShape(..., Inf) here: that is the convex hull and it
% creates an artificial straight boundary for curved swept areas.
sweptPoly = polyshape();
for i = 1:N
    V = getVehicleBox(xs(i), ys(i), yaws(i), LF, lr, hlb);
    sweptPoly = union(sweptPoly, polyshape(V(:,1), V(:,2), 'Simplify', false));
end
sweptPoly = simplify(sweptPoly);

%% Plot
figure('Color',[0.85 0.85 0.85], 'Position', [100 100 700 700]);
hold on; box on;
axis equal;
daspect([1 1 1]);
xlim([-2 3]);
ylim([-2 4]);
set(gca, 'FontSize', 12);

[sweptX, sweptY] = boundary(sweptPoly);
patch(sweptX, sweptY, [0.78 0.86 0.94], ...
    'EdgeColor','none', ...
    'FaceAlpha',0.85);

t1Color = [0.85 0.45 0.10];

% Vehicle box at t
V0 = getVehicleBox(x0, y0, yaw0, LF, lr, hlb);
patch(V0(:,1), V0(:,2), [0.85 0.9 0.9], ...
    'EdgeColor',[0 0.75 0.6], ...
    'LineWidth',2);

% Vehicle box at t1
V1 = getVehicleBox(x1, y1, yaw1, LF, lr, hlb);
patch(V1(:,1), V1(:,2), 'w', ...
    'FaceColor','none', ...
    'EdgeColor', t1Color, ...
    'LineStyle','--', ...
    'LineWidth',0.8);

% Front-wheel steering angle
phy0 = 0.5;
phy1 = phy0;

DrawWheelsStyled(x0, y0, yaw0, phy0, 'solid', []);
DrawWheelsStyled(x1, y1, yaw1, phy1, 'dashed', t1Color);

% Red embodiment box containing the two vehicle poses
allCorners = [V0; V1];

expand_right = 0.18;
expand_left = 0;

xmin_box = min(allCorners(:,1)) - expand_left;
xmax_box = max(allCorners(:,1)) + expand_right;
ymin_box = min(allCorners(:,2));
ymax_box = max(allCorners(:,2));

embodyBox = [xmin_box, ymin_box;
             xmax_box, ymin_box;
             xmax_box, ymax_box;
             xmin_box, ymax_box];

patch(embodyBox(:,1), embodyBox(:,2), 'w', ...
    'FaceColor','none', ...
    'EdgeColor',[0.90 0.05 0.05], ...
    'LineWidth',2.2);

% Rear-axle trajectory
plot(xs, ys, 'k--', 'LineWidth',1.2);
drawTrajectoryArrows(xs, ys, 1, 0.32, 'k');

% Rear-axle centers
plot(x0, y0, 'ko', 'MarkerFaceColor','k', 'MarkerSize',6);
plot(x1, y1, 'ko', 'MarkerFaceColor','k', 'MarkerSize',6);

% Export an editable vector graphic for Visio. The same file is overwritten
% every time this script runs.
set(gcf, 'Renderer', 'painters');
drawnow;
print(gcf, '-dmeta', '-painters', fullfile(pwd, 'draw.emf'));

%% Local functions
function V = getVehicleBox(x, y, yaw, LF, lr, hlb)
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

function drawHeading(x, y, yaw, L, colorv) %#ok<DEFNU>
    quiver(x, y, L*cos(yaw), L*sin(yaw), 0, ...
        'Color', colorv, 'LineWidth',1.8, 'MaxHeadSize',0.25);
end

function drawTrajectoryArrows(xs, ys, numArrows, arrowLen, colorv)
if numArrows == 1
    idx = round(0.5 * (numel(xs) - 1)) + 1;
else
    idx = round(linspace(0.25, 0.75, numArrows) * (numel(xs) - 1)) + 1;
end
idx = min(max(idx, 2), numel(xs) - 1);

for j = 1:numel(idx)
    i = idx(j);
    tangent = [xs(i+1) - xs(i-1), ys(i+1) - ys(i-1)];
    tangentNorm = hypot(tangent(1), tangent(2));
    if tangentNorm < eps
        continue;
    end

    tangent = arrowLen * tangent / tangentNorm;
    quiver(xs(i) - 0.5*tangent(1), ys(i) - 0.5*tangent(2), ...
        tangent(1), tangent(2), 0, ...
        'Color', colorv, ...
        'LineWidth', 2.0, ...
        'MaxHeadSize', 2.4, ...
        'AutoScale', 'off');
end
end

function DrawWheelsStyled(x, y, theta, phy, mode, customColor)
global params

lw  = params.vehicle.lw;
hlb = params.vehicle.hlb;

wheel_len = 0.2;
wheel_w   = 0.05;

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
        wheelLineWidth = 1.0;
        axleLineWidth  = 1.0;
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

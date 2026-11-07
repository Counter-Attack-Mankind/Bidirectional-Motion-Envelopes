function dsa(case_id_val)
close all;
global params_
figure(1)
beisu = 1;
nfe = params_.opti.nfe;
params_.result.vidObj = VideoWriter(num2str(case_id_val));
params_.result.vidObj.Quality = 100;
params_.result.vidObj.FrameRate = round(nfe / (12.0 * beisu));
open(params_.result.vidObj);

subplot(4, 4, [1 2 3 5 6 7 9 10 11 13 14 15]);
plot3(params_.task.x0, params_.task.y0, params_.task.z0, 'Color', [1 0.07 0.65], 'Marker', 'o', 'MarkerSize', 9, 'MarkerFaceColor', [1 0.07 0.65]);
hold on;
plot3(params_.task.xg, params_.task.yg, params_.task.zg, 'Color', [0.1333 0.6941 0.2980], 'Marker', 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.1333 0.6941 0.2980]);
hd_libai = plot3(params_.limo.x, params_.limo.y, params_.limo.z, 'LineWidth', 2, 'Color', [0.75 0.75 0.75]);
set(gcf, 'outerposition', get(0,'screensize'));

for ii = 1 : params_.environment.nobs
    elem = params_.task.obs(ii);
    DrawBall(elem.x, elem.y, elem.z, elem.r*0.9); hold on;
end
set(gcf, 'outerposition', get(0,'screensize'));
axis equal;
light;
% title(['Case # ', num2str(case_id_val)]);
global rand_angle0
rand_angle0 = rand * 360;
view(rand_angle0, 30);

zlim([0, max([params_.limo.z + 0.01, params_.task.zg + 0.01])]);

xlabel('X [m]', 'FontName', 'Arial Narrow', 'FontSize', 15, 'FontWeight', 'bold');
ylabel('Y [m]', 'FontName', 'Arial Narrow', 'FontSize', 15, 'FontWeight', 'bold');
zlabel('Z [m]', 'FontName', 'Arial Narrow', 'FontSize', 15, 'FontWeight', 'bold');
ax = gca; ax.FontName = 'Arial Narrow'; ax.FontSize = 15; ax.FontWeight = 'bold';
grid minor; box on;

drawnow;

subplot(4, 4, 4);
timeline = linspace(0, params_.limo.tf, nfe);
plot(timeline, params_.limo.v, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
grid minor; hold on; box on;
xlabel('Time [s]');
ylabel('Needle Insertion Velocity [m/s]');
xlim([0 max(timeline)]);
ylim([min(params_.limo.v) - 0.001, max(params_.limo.v) + 0.001]);
ax = gca; ax.FontName = 'Arial Narrow'; ax.FontSize = 12; ax.FontWeight = 'bold';

subplot(4, 4, 8);
plot(timeline, params_.limo.w, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
grid minor; hold on; box on;
xlabel('Time [s]');
ylabel('Needle Roll Rate [rad/s]');
xlim([0 max(timeline)]);
ylim([min(params_.limo.w) - 0.01, max(params_.limo.w) + 0.01]);
ax = gca; ax.FontName = 'Arial Narrow'; ax.FontSize = 12; ax.FontWeight = 'bold';

subplot(4, 4, 12);
plot(timeline, params_.limo.alpha, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
grid minor; hold on; box on;
xlabel('Time [s]');
ylabel('Needle Yaw Angle [rad]');
xlim([0 max(timeline)]);
ylim([min(params_.limo.alpha) - 0.1, max(params_.limo.alpha) + 0.1]);
ax = gca; ax.FontName = 'Arial Narrow'; ax.FontSize = 12; ax.FontWeight = 'bold';

subplot(4, 4, 16);
plot(timeline, params_.limo.beta, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
grid minor; hold on; box on;
xlabel('Time [s]');
ylabel('Needle Pitch Angle [rad]');
xlim([0 max(timeline)]);
ylim([min(params_.limo.beta) - 0.1, max(params_.limo.beta) + 0.1]);
ax = gca; ax.FontName = 'Arial Narrow'; ax.FontSize = 12; ax.FontWeight = 'bold';



for q = 1 : round(params_.result.vidObj.FrameRate * 0.8)
    writeVideo(params_.result.vidObj, getframe(gcf));
end

id_list = 1 : beisu : nfe;
for q = 1 : length(id_list)
    ii = id_list(q);
    handles = [];
    subplot(4, 4, [1 2 3 5 6 7 9 10 11 13 14 15]);
    angle = ii / nfe * 72;
    kappa = params_.machine.kappa;
    hd = plotNeedleState(params_.limo.x(ii), params_.limo.y(ii), params_.limo.z(ii), params_.limo.v(ii), params_.limo.alpha(ii), params_.limo.beta(ii), params_.limo.gammar(ii), kappa, angle);
    if (q > 1)
        delete(hd_libai);
        id_pre = id_list(q - 1);
        id_cur = id_list(q);
        plot3(params_.limo.x(id_pre : id_cur), params_.limo.y(id_pre : id_cur), params_.limo.z(id_pre : id_cur), 'Color', [0 162 232] ./ 255, 'LineWidth', 5);
        hd_libai = plot3(params_.limo.x(id_cur:end), params_.limo.y(id_cur:end), params_.limo.z(id_cur:end), 'LineWidth', 2, 'Color', [0.75 0.75 0.75]);
    end
    handles = [handles, hd];
    drawnow;

    subplot(4, 4, 4);
    hd = plot(timeline(ii), params_.limo.v(ii), 'o', 'MarkerSize', 7, 'LineWidth', 2, 'Color', [240 135 132] ./ 255);
    hd2 = plot(timeline(1 : ii), params_.limo.v(1 : ii), 'LineWidth', 5, 'Color', [240 135 132] ./ 255);
    handles = [handles, hd, hd2];
    drawnow;

    subplot(4, 4, 8);
    hd = plot(timeline(ii), params_.limo.w(ii), 'o', 'MarkerSize', 7, 'LineWidth', 2, 'Color', [161 250 79] ./ 255);
    hd2 = plot(timeline(1 : ii), params_.limo.w(1 : ii), 'LineWidth', 5, 'Color', [161 250 79] ./ 255);
    handles = [handles, hd, hd2];
    drawnow;

    subplot(4, 4, 12);
    hd = plot(timeline(ii), params_.limo.alpha(ii), 'o', 'MarkerSize', 7, 'LineWidth', 2, 'Color', [0 152 232] ./ 255);
    hd2 = plot(timeline(1 : ii), params_.limo.alpha(1 : ii), 'LineWidth', 5, 'Color', [0 152 232] ./ 255);
    handles = [handles, hd, hd2];

    subplot(4, 4, 16);
    hd = plot(timeline(ii), params_.limo.beta(ii), 'o', 'MarkerSize', 7, 'LineWidth', 2, 'Color', [117 22 63] ./ 255);
    hd2 = plot(timeline(1 : ii), params_.limo.beta(1 : ii), 'LineWidth', 5, 'Color', [117 22 63] ./ 255);
    handles = [handles, hd, hd2];

    drawnow;
    writeVideo(params_.result.vidObj, getframe(gcf));

    if (ii == nfe)
        for qq = 1 : round(params_.result.vidObj.FrameRate * 0.8)
            writeVideo(params_.result.vidObj, getframe(gcf));
        end
    end

    for jj = 1 : length(handles)
        delete(handles(jj));
    end
end
close(params_.result.vidObj);
end

function handles = plotNeedleState(x, y, z, v, alpha, beta, gamma, kappa, angle)
dirWorld = [cos(alpha)*cos(beta); sin(alpha)*cos(beta); sin(beta)];
dirWorld = dirWorld ./ norm(dirWorld + 1e-12);

hd1 = plot3(x, y, z, 'ro', 'MarkerSize', 5, 'LineWidth', 2);
arrowLen = 2.0 * v;
hd2 = quiver3(x, y, z, arrowLen*dirWorld(1), arrowLen*dirWorld(2), arrowLen*dirWorld(3), 'Color', 'k', 'LineWidth', 5, 'MaxHeadSize', 2);

bendWorld2 = [cos(alpha-0.5*pi); sin(alpha-0.5*pi); 0];
bendWorld3 = rodriguesRotate(bendWorld2, dirWorld, -gamma);
hd3 = quiver3(x, y, z, arrowLen*bendWorld3(1), arrowLen*bendWorld3(2), arrowLen*bendWorld3(3), 'Color','g', 'LineWidth', 4, 'MaxHeadSize',2);

hd4 = plotPlaneThroughPoint(x, y, z, bendWorld3(1), bendWorld3(2), bendWorld3(3));

bendWorld4 = rodriguesRotate(bendWorld3, dirWorld, -pi/2);
hd6 = quiver3(x, y, z, arrowLen*bendWorld4(1), arrowLen*bendWorld4(2), arrowLen*bendWorld4(3), 'Color','r', 'LineWidth', 4, 'MaxHeadSize',2);

arcXYZ = plotArcInPlane(x, y, z, dirWorld(1), dirWorld(2), dirWorld(3), bendWorld3(1), bendWorld3(2), bendWorld3(3), kappa);
for ii = 1 : 20
    [xNew, yNew, zNew] = rotatePointAroundLine(arcXYZ(1,ii), arcXYZ(2,ii), arcXYZ(3,ii), dirWorld(1), dirWorld(2), dirWorld(3), x, y, z, pi);
    arcXYZ(1, ii) = xNew;
    arcXYZ(2, ii) = yNew;
    arcXYZ(3, ii) = zNew;
end
hd5 = plot3(arcXYZ(1,:), arcXYZ(2,:), arcXYZ(3,:), 'r:', 'LineWidth', 2);
handles = [hd1, hd2, hd3, hd4, hd5, hd6];
global rand_angle0
view(rand_angle0 + angle, 30);
end

function vOut = rodriguesRotate(vIn, axisUnit, angle)
c=cos(angle); s=sin(angle);
dotVal= dot(axisUnit,vIn);
crossVal= cross(axisUnit,vIn);
vOut= vIn*c + crossVal*s + axisUnit*(dotVal*(1-c));
end

function handle = plotPlaneThroughPoint(x, y, z, p1, p2, p3)
normalVec = [p1; p2; p3];
nrm = norm(normalVec);
if nrm < 1e-12
    error('平面法向量过小或为零，无法定义平面.');
end
normalVec = normalVec / nrm;

baseRef = [0; 0; 1];
if abs(dot(baseRef, normalVec))>0.99
    baseRef = [0;1;0];
end
e1 = cross(normalVec, baseRef);
e1 = e1 / norm(e1 + 1e-12);

e2 = cross(normalVec, e1);
e2 = e2 / norm(e2 + 1e-12);

planeSide = 0.02;
corner2D = [ -planeSide,  planeSide,  planeSide, -planeSide;
    -planeSide, -planeSide,  planeSide,  planeSide ];

planeXYZ = zeros(3,4);
for i = 1 : 4
    pxLocal = corner2D(1,i);
    pyLocal = corner2D(2,i);
    planeXYZ(:,i) = [x;y;z] + pxLocal*e1 + pyLocal*e2;
end

handle = patch('XData', planeXYZ(1,:), 'YData', planeXYZ(2,:), 'ZData', planeXYZ(3,:),  'FaceColor', [1 0 0],'FaceAlpha',0.2,'EdgeColor','none');
end

function [xNew, yNew, zNew] = rotatePointAroundLine(x, y, z, p1, p2, p3, xc, yc, zc, ang)
v = [x - xc; y - yc; z - zc];
p = [p1; p2; p3];
p_norm = norm(p);
if p_norm < 1e-12
    error('方向向量(p1,p2,p3)长度过小，无法进行旋转！');
end
u = p / p_norm;

cosA = cos(ang);
sinA = sin(ang);
v_rot = v*cosA + cross(u, v)*sinA + u*(dot(u,v))*(1 - cosA);

xNew = v_rot(1) + xc;
yNew = v_rot(2) + yc;
zNew = v_rot(3) + zc;
end

function arcXYZ = plotArcInPlane(x, y, z, px, py, pz, p1, p2, p3, kappa)
F = [x; y; z];
P = [px; py; pz];
Q = [p1; p2; p3];

if norm(P)<1e-12
    error('向量P 过小, 无法当切线方向');
end
if norm(Q)<1e-12
    error('向量Q 过小, 无法当平面法向量');
end

normalVec = Q / norm(Q);

planeSide = 0.05;  % 可调

baseRef = [0;1;0];
tempCross = cross(normalVec, baseRef);
if norm(tempCross)<1e-9
    baseRef = [0;0;1];
    tempCross = cross(normalVec, baseRef);
end
e1 = tempCross / norm(tempCross + 1e-12);
e2 = cross(normalVec, e1);
e2 = e2 / norm(e2 + 1e-12);

corner2D = [ -planeSide,  planeSide,  planeSide, -planeSide;
    -planeSide, -planeSide,  planeSide,  planeSide];  % 2×4
planeXYZ = zeros(3,4);
for i=1:4
    pxLocal = corner2D(1,i);
    pyLocal = corner2D(2,i);
    planeXYZ(:,i)= F + pxLocal* e1 + pyLocal* e2;
end

Punit = P / norm(P);
Qunit= normalVec;  % = Q / |Q|
crossVal = cross(Qunit, Punit);
if norm(crossVal)<1e-12
    warning('P 可能与 Q 平行, 无法定义弯曲');
    return
end
u = crossVal / norm(crossVal);
R = 1.0 / kappa;
C = F - R * u;
arcDeg = 30;   % 60
arcRad= deg2rad(arcDeg);
nArc= 20;
tArc= linspace(0, arcRad, nArc);
arcXYZ= zeros(3,nArc);
for j=1:nArc
    th= tArc(j);
    arcXYZ(:,j)= C + R*cos(th)*u + R*sin(th)*Punit;
end
% 若出现方向相反,可检查 dot( arcXYZ(:,2)-arcXYZ(:,1), P )<0 => flip
if dot(arcXYZ(:,2)-arcXYZ(:,1), P)<0
    % flip
    C = F + R*u;  % 改成 +R*u
    for j=1:nArc
        th= tArc(j);
        arcXYZ(:,j)= C + R*cos(th)*(-u) + R*sin(th)*Punit;
    end
end
end
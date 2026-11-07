function Final_Viusalize()

global params

%% ======== 是否使用 enriched 位姿序列 ========
use_enriched = isfield(params,'ef') && ...
               isfield(params.ef,'enriched_x') && ...
               ~isempty(params.ef.enriched_x);

%% =========================================================
%  1) 位姿序列：允许插值（用于平滑动画）
%  2) 具身盒子尺寸：不插值，采用原始采样点零阶保持
%% =========================================================

if use_enriched
    % -------- 平滑动画位姿 --------
    x  = params.ef.enriched_x(:);
    y  = params.ef.enriched_y(:);
    th = params.ef.enriched_theta(:);

    if isfield(params.ef,'enriched_phy') && ~isempty(params.ef.enriched_phy)
        phy = params.ef.enriched_phy(:);
    elseif isfield(params.ef,'phy') && ~isempty(params.ef.phy)
        phy = params.ef.phy(:);
    else
        phy = zeros(size(x));
    end

    % -------- 原始具身盒子（不插值）--------
    x_box  = params.ef.x(:);
    y_box  = params.ef.y(:);
    th_box = params.ef.theta(:);

    up_box    = pickField(params.ef,'up','a',zeros(size(x_box)));
    left_box  = pickField(params.ef,'left','b',zeros(size(x_box)));
    down_box  = pickField(params.ef,'down','c',zeros(size(x_box)));
    right_box = pickField(params.ef,'right','d',zeros(size(x_box)));

    up_box    = up_box(:);
    left_box  = left_box(:);
    down_box  = down_box(:);
    right_box = right_box(:);

else
    % -------- 无 enriched：全部用原始序列 --------
    x  = params.ef.x(:);
    y  = params.ef.y(:);
    th = params.ef.theta(:);

    if isfield(params.ef,'phy') && ~isempty(params.ef.phy)
        phy = params.ef.phy(:);
    else
        phy = zeros(size(x));
    end

    x_box  = x;
    y_box  = y;
    th_box = th;

    up_box    = pickField(params.ef,'up','a',zeros(size(x_box)));
    left_box  = pickField(params.ef,'left','b',zeros(size(x_box)));
    down_box  = pickField(params.ef,'down','c',zeros(size(x_box)));
    right_box = pickField(params.ef,'right','d',zeros(size(x_box)));

    up_box    = up_box(:);
    left_box  = left_box(:);
    down_box  = down_box(:);
    right_box = right_box(:);
end

%% ======== 长度统一 ========
n = min([numel(x), numel(y), numel(th), numel(phy)]);
x   = x(1:n);
y   = y(1:n);
th  = th(1:n);
phy = phy(1:n);

nbox = min([numel(x_box), numel(y_box), numel(th_box), ...
            numel(up_box), numel(left_box), numel(down_box), numel(right_box)]);
x_box     = x_box(1:nbox);
y_box     = y_box(1:nbox);
th_box    = th_box(1:nbox);
up_box    = up_box(1:nbox);
left_box  = left_box(1:nbox);
down_box  = down_box(1:nbox);
right_box = right_box(1:nbox);

if n < 2
    warning('Trajectory too short.');
    return;
end

if nbox < 1
    warning('Embodiment box data is empty.');
    return;
end

%% ======== 建立"每一帧对应哪个原始具身盒子"的索引 ========
% 原理：
% 对于每个动画帧 k（enriched pose），找到它最接近的原始采样点索引；
% 再做"前向保持"分段：从原始点 i 到 i+1 之间都使用第 i 个具身盒子。
%
% 这样具身盒子不会连续缩放，只会在下一个原始盒子处跳变。

if use_enriched
    boxIdxOfFrame = buildBoxHoldIndex(x, y, th, x_box, y_box, th_box);
else
    boxIdxOfFrame = (1:n)';
end

%% ======== 动画节奏 ========
fps = 48;
dt_frame = 1/fps;
speed = 1.0;

%% ======== 车辆几何参数 ========
lf  = params.vehicle.lf;
lw  = params.vehicle.lw;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% ======== 创建 Figure ========
fig = figure(randi(10000));
set(fig,'OuterPosition',get(0,'ScreenSize'));

ax = axes(fig);
hold(ax,'on');
grid(ax,'minor');
box(ax,'on');
axis(ax,'equal');

axis(ax,[params.environment.xmin, params.environment.xmax, ...
         params.environment.ymin, params.environment.ymax]);

set(ax,'FontWeight','bold','FontSize',18,'FontName','Arial Narrow');
xlabel(ax,'x / m');
ylabel(ax,'y / m');
title(ax,'Vehicle Animation');

legend(ax,'off');

%% ======== 障碍物 ========
for ii = 1:params.environment.num_obs
    hObs = fill(ax, ...
        params.environment.obs(ii).x, ...
        params.environment.obs(ii).y, ...
        [0.75 0.75 0.75], ...
        'EdgeColor','none');
    set(hObs,'HandleVisibility','off');
end

%% ======== 全局轨迹参考（不进 legend）========
hRef = plot(ax, x, y, '-', ...
    'Color', [0.7 0.7 0.7], ...
    'LineWidth', 1.2);
set(hRef,'HandleVisibility','off');

%% ======== 尾迹（真实动画对象，不进 legend）========
trail = plot(ax, nan, nan, 'k-', 'LineWidth', 2);
trailPts = plot(ax, nan, nan, 'o', ...
    'MarkerSize', 1, ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'LineStyle', 'none');

set(trail,   'HandleVisibility','off');
set(trailPts,'HandleVisibility','off');

%% ======== 初始化车身与具身盒子 ========
[XB,YB] = body_rect_corners(x(1), y(1), th(1), lf, lw, lr, hlb);

ibox = boxIdxOfFrame(1);
[XE,YE] = ddd2_poly( ...
    x(1), y(1), th(1), ...
    up_box(ibox), left_box(ibox), down_box(ibox), right_box(ibox));

% 车身填充（真实动画对象）
bodyPatch = patch(ax, ...
    'XData', XB, 'YData', YB, ...
    'FaceColor', [0 0 0], ...
    'FaceAlpha', 0.25, ...
    'EdgeColor', 'none');
set(bodyPatch,'HandleVisibility','off');

% 车身边框
bodyEdge = patch(ax, ...
    'XData', XB, 'YData', YB, ...
    'FaceColor', 'none', ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.2);
set(bodyEdge,'HandleVisibility','off');

% 具身盒子填充（真实动画对象）
embPatch = patch(ax, ...
    'XData', XE, 'YData', YE, ...
    'FaceColor', [0 162 232]/255, ...
    'FaceAlpha', 0.15, ...
    'EdgeColor', 'none');
set(embPatch,'HandleVisibility','off');

% 具身盒子边框
embEdge = patch(ax, ...
    'XData', XE, 'YData', YE, ...
    'FaceColor', 'none', ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.0);
set(embEdge,'HandleVisibility','off');

%% ======== 扫掠区域初始化 ========
sweptPoly = polyshape();

hSweep = plot(polyshape(), ...
    'FaceColor', [0 162 232]/255, ...
    'FaceAlpha', 0.35, ...
    'EdgeColor', 'none');
set(hSweep,'HandleVisibility','off');

%% ======== 轮子初始化（不进 legend）========
[h1,h2,h3,h4,h5,h6] = DrawWheels(x(1), y(1), th(1), phy(1));
hW = [h1 h2 h3 h4 h5 h6];
set(hW,'HandleVisibility','off');

%% ======== 先调坐标轴位置，再建 legend ========
ax.Position = [0.06 0.08 0.68 0.86];

%% ======== 专用 legend 虚拟句柄（避免错位）========
legBody = patch(nan, nan, [0 0 0], ...
    'FaceAlpha', 0.25, ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.2);

legEmb = patch(nan, nan, [0 162 232]/255, ...
    'FaceAlpha', 0.15, ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.0);

legTrail = plot(nan, nan, 'k-', 'LineWidth', 2);

legPts = plot(nan, nan, 'o', ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'LineStyle', 'none');

lab = {'Vehicle body','Embodiment box','Traversed path','Samples'};
lgd = legend(ax, [legBody, legEmb, legTrail, legPts], lab, ...
    'Location', 'eastoutside');
lgd.AutoUpdate = 'off';
lgd.FontSize = 12;
lgd.Box = 'on';

%% ======== 右上角信息框（轴外）========
infoTxt = text(ax, 1.02, 0.98, '', ...
    'Units', 'normalized', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top', ...
    'FontSize', 13, ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w', ...
    'Margin', 14, ...
    'EdgeColor', [0.6 0.6 0.6]);

%% ======== 动画循环 ========
for k = 1:n

    % ===== 尾迹更新 =====
    set(trail,   'XData', x(1:k), 'YData', y(1:k));
    set(trailPts,'XData', x(1:k), 'YData', y(1:k));

    % ===== 更新车身（位姿可平滑插值）=====
    [XB,YB] = body_rect_corners(x(k), y(k), th(k), lf, lw, lr, hlb);
    set(bodyPatch, 'XData', XB, 'YData', YB);
    set(bodyEdge , 'XData', XB, 'YData', YB);

    % ===== 扫掠区域更新（按真实车身）=====
    bodyPoly = polyshape(XB, YB);
    sweptPoly = union(sweptPoly, bodyPoly);
    set(hSweep, 'Shape', sweptPoly);

    % ===== 更新具身盒子（尺寸零阶保持，不插值）=====
    ibox = boxIdxOfFrame(k);

    [XE,YE] = ddd2_poly( ...
        x(k), y(k), th(k), ...
        up_box(ibox), left_box(ibox), down_box(ibox), right_box(ibox));

    set(embPatch, 'XData', XE, 'YData', YE);
    set(embEdge , 'XData', XE, 'YData', YE);

    % ===== 更新轮子 =====
    if all(isgraphics(hW))
        delete(hW);
    end
    [h1,h2,h3,h4,h5,h6] = DrawWheels(x(k), y(k), th(k), phy(k));
    hW = [h1 h2 h3 h4 h5 h6];
    set(hW,'HandleVisibility','off');

    % ===== 信息框 =====
    set(infoTxt,'String',sprintf([ ...
        'Frame = %d / %d\n\n' ...
        'X = %.2f,  Y = %.2f\n\n' ...
        'Theta = %.2f rad  (%.1f deg)\n\n' ...
        'Box sample = %d / %d\n\n' ...
        'e_{up}    = %.3f\n' ...
        'e_{left}  = %.3f\n' ...
        'e_{down}  = %.3f\n' ...
        'e_{right} = %.3f'], ...
        k, n, ...
        x(k), y(k), ...
        th(k), th(k)*180/pi, ...
        ibox, nbox, ...
        up_box(ibox), left_box(ibox), down_box(ibox), right_box(ibox)));

    drawnow limitrate
    pause(dt_frame/speed)
end

end

%% ======== 为每一帧建立"保持到下一个盒子"的索引 ========
function boxIdxOfFrame = buildBoxHoldIndex(x, y, th, x_box, y_box, th_box)

n  = numel(x);
nb = numel(x_box);

% 每个原始盒子点，在 enriched 序列中找最近帧
anchor = zeros(nb,1);

for i = 1:nb
    dpos = (x - x_box(i)).^2 + (y - y_box(i)).^2;

    % 给角度差一点权重，避免回头路时仅靠位置匹配出错
    dth = wrapToPiLocal(th - th_box(i));
    score = dpos + 0.05*(dth.^2);

    [~, anchor(i)] = min(score);
end

% 保证单调不减
anchor = cummax(anchor);
anchor(anchor < 1) = 1;
anchor(anchor > n) = n;

% 若有重复锚点，后一个至少不小于前一个
for i = 2:nb
    if anchor(i) < anchor(i-1)
        anchor(i) = anchor(i-1);
    end
end

boxIdxOfFrame = ones(n,1);

% 从 anchor(i) 到 anchor(i+1)-1 使用第 i 个盒子
for i = 1:nb-1
    k1 = anchor(i);
    k2 = anchor(i+1) - 1;
    if k2 < k1
        k2 = k1;
    end
    boxIdxOfFrame(k1:k2) = i;
end

% 最后一个盒子保持到结束
boxIdxOfFrame(anchor(nb):n) = nb;

% 对开头可能未覆盖部分，使用第一个盒子
boxIdxOfFrame(1:anchor(1)) = 1;

end

%% ======== 角度 wrap 到 [-pi, pi] ========
function a = wrapToPiLocal(a)
a = mod(a + pi, 2*pi) - pi;
end

%% ======== 车身矩形 ========
function [X,Y] = body_rect_corners(x,y,theta,lf,lw,lr,hlb)
c = cos(theta);
s = sin(theta);

AX = x + (lf+lw)*c - hlb*s;
AY = y + (lf+lw)*s + hlb*c;

BX = x + (lf+lw)*c + hlb*s;
BY = y + (lf+lw)*s - hlb*c;

CX = x - lr*c + hlb*s;
CY = y - lr*s - hlb*c;

DX = x - lr*c - hlb*s;
DY = y - lr*s + hlb*c;

X = [AX BX CX DX AX];
Y = [AY BY CY DY AY];
end

%% ======== ddd2 封装 ========
function [X,Y] = ddd2_poly(x,y,theta,a,b,c,d)
[AX,AY,BX,BY,CX,CY,DX,DY] = ddd2(x,y,theta,a,b,c,d);
X = [AX BX CX DX AX];
Y = [AY BY CY DY AY];
end

%% ======== 字段兼容读取 ========
function val = pickField(S, newName, oldName, defaultVal)
if isfield(S,newName) && ~isempty(S.(newName))
    val = S.(newName);
elseif isfield(S,oldName) && ~isempty(S.(oldName))
    val = S.(oldName);
else
    val = defaultVal;
end
end
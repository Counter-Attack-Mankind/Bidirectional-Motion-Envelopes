function Final_Viusalize_withplot()

global params
%% ======== 视频保存开关 ========
save_video = false;   % true：保存视频 / false：只显示动画
video_path = fullfile(pwd, 'results');   % 保存目录（可改）
video_name = '16.mp4'; % 文件名

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
if use_enriched
    boxIdxOfFrame = buildBoxHoldIndex(x, y, th, x_box, y_box, th_box);
else
    boxIdxOfFrame = (1:n)';
end

%% ======== 将四个外扩尺度映射到每一帧 ========
up_frame    = up_box(boxIdxOfFrame);
left_frame  = left_box(boxIdxOfFrame);
down_frame  = down_box(boxIdxOfFrame);
right_frame = right_box(boxIdxOfFrame);

timeline = 1:n;

%% ======== 速度序列（用于前进/后退配色）========
if use_enriched
    % 1) 优先使用 enriched_v
    if isfield(params.ef,'enriched_v') && ~isempty(params.ef.enriched_v)
        v_raw = params.ef.enriched_v(:);
        nv = min(numel(v_raw), n);
        v = zeros(n,1);
        v(1:nv) = v_raw(1:nv);
        if nv < n
            v(nv+1:end) = v_raw(nv);
        end

    % 2) 否则若有原始 v，则按 boxIdxOfFrame 零阶保持映射到每一帧
    elseif isfield(params.ef,'v') && ~isempty(params.ef.v)
        v_box = params.ef.v(:);
        nvb = min(numel(v_box), nbox);
        v_box = v_box(1:nvb);

        if nvb < nbox
            v_box(end+1:nbox) = v_box(end);
        end

        v = v_box(boxIdxOfFrame);

    % 3) 再不行才用位姿差分近似符号
    else
        dx = [0; diff(x)];
        dy = [0; diff(y)];
        v  = dx .* cos(th) + dy .* sin(th);
    end

else
    if isfield(params.ef,'v') && ~isempty(params.ef.v)
        v_raw = params.ef.v(:);
        nv = min(numel(v_raw), n);
        v = zeros(n,1);
        v(1:nv) = v_raw(1:nv);
        if nv < n
            v(nv+1:end) = v_raw(nv);
        end
    else
        dx = [0; diff(x)];
        dy = [0; diff(y)];
        v  = dx .* cos(th) + dy .* sin(th);
    end
end
%% ======== 动画节奏 ========
if isfield(params,'ef') && isfield(params.ef,'vis_fps') && ~isempty(params.ef.vis_fps)
    fps = params.ef.vis_fps;
else
    fps = 30;
end

if isfield(params,'ef') && isfield(params.ef,'vis_speed') && ~isempty(params.ef.vis_speed)
    speed = params.ef.vis_speed;
else
    speed = 1.0;
end

dt_frame = 1 / fps;
%% ======== 车辆几何参数 ========
lf  = params.vehicle.lf;
lw  = params.vehicle.lw;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% ======== 配色 ========
% 车辆：浅灰填充 + 黑边
vehFillColor = [0.75 0.75 0.75];
vehEdgeColor = [0 0 0];

% 具身盒子：前进蓝 / 后退红（保持当前风格，不刺眼）
embBlueFill = [0 162 232] / 255;
embBlueEdge = [0 162 232] / 255;

embRedFill  = [210 95 95] / 255;
embRedEdge  = [210 95 95] / 255;

% 右侧四图：学术风格低饱和
c_up    = [0.33 0.52 0.72];
c_down  = [0.55 0.42 0.36];
c_right = [0.46 0.46 0.62];
c_left  = [0.40 0.58 0.40];

%% ======== 创建 Figure ========
fig = figure(randi(10000));

if save_video
    if ~exist(video_path, 'dir')
        mkdir(video_path);
    end

    video_fullpath = fullfile(video_path, video_name);

    vWriter = VideoWriter(video_fullpath, 'MPEG-4');
    vWriter.FrameRate = fps;   % 和动画一致
    open(vWriter);
end

set(fig, 'Units', 'pixels', 'Position', [80 60 1600 900]);

% ===== 左侧主动画轴 =====
ax = axes('Parent',fig,'Position',[0.06 0.08 0.62 0.86]);
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

%% ======== 尾迹 ========
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

% 当前速度决定具身盒子颜色
if v(1) > 0
    curEmbFill = embBlueFill;
    curEmbEdge = embBlueEdge;
elseif v(1) < 0
    curEmbFill = embRedFill;
    curEmbEdge = embRedEdge;
else
    curEmbFill = embBlueFill;
    curEmbEdge = embBlueEdge;
end

% 车身填充
bodyPatch = patch(ax, ...
    'XData', XB, 'YData', YB, ...
    'FaceColor', vehFillColor, ...
    'FaceAlpha', 0.55, ...
    'EdgeColor', 'none');
set(bodyPatch,'HandleVisibility','off');

% 车身边框
bodyEdge = patch(ax, ...
    'XData', XB, 'YData', YB, ...
    'FaceColor', 'none', ...
    'EdgeColor', vehEdgeColor, ...
    'LineWidth', 0.8);
set(bodyEdge,'HandleVisibility','off');

% 具身盒子填充
embPatch = patch(ax, ...
    'XData', XE, 'YData', YE, ...
    'FaceColor', curEmbFill, ...
    'FaceAlpha', 0.15, ...
    'EdgeColor', 'none');
set(embPatch,'HandleVisibility','off');

% 具身盒子边框
embEdge = patch(ax, ...
    'XData', XE, 'YData', YE, ...
    'FaceColor', 'none', ...
    'EdgeColor', curEmbEdge, ...
    'LineWidth', 1.0);
set(embEdge,'HandleVisibility','off');

%% ======== 扫掠区域初始化 ========
sweptPolyFwd = polyshape();
sweptPolyRev = polyshape();

hSweepFwd = plot(polyshape(), ...
    'FaceColor', embBlueFill, ...
    'FaceAlpha', 0.16, ...
    'EdgeColor', 'none');
set(hSweepFwd,'HandleVisibility','off');

hSweepRev = plot(polyshape(), ...
    'FaceColor', embRedFill, ...
    'FaceAlpha', 0.16, ...
    'EdgeColor', 'none');
set(hSweepRev,'HandleVisibility','off');

%% ======== 轮子初始化 ========
[h1,h2,h3,h4,h5,h6] = DrawWheels_improve(ax, x(1), y(1), th(1), phy(1));
hW = [h1 h2 h3 h4 h5 h6];
set(hW,'HandleVisibility','off');

%% ======== legend ========
legBody = patch(nan, nan, vehFillColor, ...
    'FaceAlpha', 0.55, ...
    'EdgeColor', vehEdgeColor, ...
    'LineWidth', 0.8);

legEmb = patch(nan, nan, embBlueFill, ...
    'FaceAlpha', 0.15, ...
    'EdgeColor', embBlueEdge, ...
    'LineWidth', 1.0);

legTrail = plot(nan, nan, 'k-', 'LineWidth', 2);
legPts = plot(nan, nan, 'o', ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', 'r', ...
    'MarkerEdgeColor', 'r', ...
    'LineStyle', 'none');

lab = {'Vehicle body','Embodiment box','Traversed path','Samples'};
lgd = legend(ax, [legBody, legEmb, legTrail, legPts], lab, ...
    'Location', 'southoutside', ...
    'Orientation', 'horizontal');
lgd.AutoUpdate = 'off';
lgd.FontSize = 12;
lgd.Box = 'on';

%% ======== 右侧四个外扩尺度子图 ========
right_x = 0.65;
right_w = 0.24;
hgt     = 0.155;
gap     = 0.045;

y1 = 0.79;
y2 = y1 - hgt - gap;
y3 = y2 - hgt - gap;
y4 = y3 - hgt - gap;

ax_up    = axes('Parent',fig,'Position',[right_x y1 right_w hgt]);
ax_down  = axes('Parent',fig,'Position',[right_x y2 right_w hgt]);
ax_right = axes('Parent',fig,'Position',[right_x y3 right_w hgt]);
ax_left  = axes('Parent',fig,'Position',[right_x y4 right_w hgt]);

pad_up    = max(1e-3, 0.08*(max(up_frame)-min(up_frame)+1e-6));
pad_down  = max(1e-3, 0.08*(max(down_frame)-min(down_frame)+1e-6));
pad_right = max(1e-3, 0.08*(max(right_frame)-min(right_frame)+1e-6));
pad_left  = max(1e-3, 0.08*(max(left_frame)-min(left_frame)+1e-6));

% 前三张图不显示 x 轴标题，只保留最下面一张
setupRightAxis(ax_up,    n, min(up_frame)-pad_up,       max(up_frame)+pad_up,       '',         'e_{up} / m');
setupRightAxis(ax_down,  n, min(down_frame)-pad_down,   max(down_frame)+pad_down,   '',         'e_{down} / m');
setupRightAxis(ax_right, n, min(right_frame)-pad_right, max(right_frame)+pad_right, '',         'e_{right} / m');
setupRightAxis(ax_left,  n, min(left_frame)-pad_left,   max(left_frame)+pad_left,   'Frame',    'e_{left} / m');

% 上面三张图隐藏 x 轴刻度标签，让整体更干净
set(ax_up,    'XTickLabel', []);
set(ax_down,  'XTickLabel', []);
set(ax_right, 'XTickLabel', []);
% 不预先画全曲线，只随时间推进
hUpLine    = plot(ax_up,    NaN, NaN, '-', 'Color', c_up,    'LineWidth', 2.0);
hDownLine  = plot(ax_down,  NaN, NaN, '-', 'Color', c_down,  'LineWidth', 2.0);
hRightLine = plot(ax_right, NaN, NaN, '-', 'Color', c_right, 'LineWidth', 2.0);
hLeftLine  = plot(ax_left,  NaN, NaN, '-', 'Color', c_left,  'LineWidth', 2.0);

hUpPt    = plot(ax_up,    NaN, NaN, 'o', 'Color', c_up,    'MarkerFaceColor', c_up,    'MarkerSize', 5);
hDownPt  = plot(ax_down,  NaN, NaN, 'o', 'Color', c_down,  'MarkerFaceColor', c_down,  'MarkerSize', 5);
hRightPt = plot(ax_right, NaN, NaN, 'o', 'Color', c_right, 'MarkerFaceColor', c_right, 'MarkerSize', 5);
hLeftPt  = plot(ax_left,  NaN, NaN, 'o', 'Color', c_left,  'MarkerFaceColor', c_left,  'MarkerSize', 5);

%% ======== 动画循环 ========
for k = 1:n

    % ===== 尾迹更新 =====
    set(trail,   'XData', x(1:k), 'YData', y(1:k));
    set(trailPts,'XData', x(1:k), 'YData', y(1:k));

    % ===== 更新车身 =====
    [XB,YB] = body_rect_corners(x(k), y(k), th(k), lf, lw, lr, hlb);
    set(bodyPatch, 'XData', XB, 'YData', YB);
    set(bodyEdge , 'XData', XB, 'YData', YB);

    % ===== 扫掠区域更新 =====
    bodyPoly = polyshape(XB, YB);
    if v(k) > 0
        sweptPolyFwd = union(sweptPolyFwd, bodyPoly);
        set(hSweepFwd, 'Shape', sweptPolyFwd);
    elseif v(k) < 0
        sweptPolyRev = union(sweptPolyRev, bodyPoly);
        set(hSweepRev, 'Shape', sweptPolyRev);
    else
        sweptPolyFwd = union(sweptPolyFwd, bodyPoly);
        set(hSweepFwd, 'Shape', sweptPolyFwd);
    end

    % ===== 更新具身盒子（零阶保持）=====
    ibox = boxIdxOfFrame(k);
    [XE,YE] = ddd2_poly( ...
        x(k), y(k), th(k), ...
        up_box(ibox), left_box(ibox), down_box(ibox), right_box(ibox));

    if v(k) > 0
        curEmbFill = embBlueFill;
        curEmbEdge = embBlueEdge;
    elseif v(k) < 0
        curEmbFill = embRedFill;
        curEmbEdge = embRedEdge;
    else
        curEmbFill = embBlueFill;
        curEmbEdge = embBlueEdge;
    end

    set(embPatch, 'XData', XE, 'YData', YE, ...
        'FaceColor', curEmbFill);
    set(embEdge , 'XData', XE, 'YData', YE, ...
        'EdgeColor', curEmbEdge);

    % ===== 更新轮子 =====
    if all(isgraphics(hW))
        delete(hW);
    end

    [h1,h2,h3,h4,h5,h6] = DrawWheels_improve(ax, x(k), y(k), th(k), phy(k));
    hW = [h1 h2 h3 h4 h5 h6];
    set(hW,'HandleVisibility','off');

    % ===== 右侧四图随时间描绘 =====
    set(hUpLine,    'XData', timeline(1:k), 'YData', up_frame(1:k));
    set(hDownLine,  'XData', timeline(1:k), 'YData', down_frame(1:k));
    set(hRightLine, 'XData', timeline(1:k), 'YData', right_frame(1:k));
    set(hLeftLine,  'XData', timeline(1:k), 'YData', left_frame(1:k));

    set(hUpPt,    'XData', timeline(k), 'YData', up_frame(k));
    set(hDownPt,  'XData', timeline(k), 'YData', down_frame(k));
    set(hRightPt, 'XData', timeline(k), 'YData', right_frame(k));
    set(hLeftPt,  'XData', timeline(k), 'YData', left_frame(k));

    drawnow limitrate

    if save_video
        frame = getframe(fig);
        writeVideo(vWriter, frame);
    end
    pause(dt_frame/speed)
end

if save_video
    close(vWriter);
    disp(['视频已保存到: ', video_fullpath]);
end

end

%% ======== 右侧坐标轴统一设置 ========
function setupRightAxis(axh, n, ymin_, ymax_, xlabelStr, ylabelStr)

hold(axh,'on');
box(axh,'on');
grid(axh,'on');
grid(axh,'minor');

xlim(axh,[1 n]);
ylim(axh,[ymin_ ymax_]);

xlabel(axh, xlabelStr, ...
    'FontName','Arial Narrow', ...
    'FontSize',16, ...
    'FontWeight','bold');

ylabel(axh, ylabelStr, ...
    'FontName','Arial Narrow', ...
    'FontSize',16, ...
    'FontWeight','bold', ...
    'Rotation',90);

set(axh, ...
    'FontName','Arial Narrow', ...
    'FontSize',14, ...
    'FontWeight','bold', ...
    'LineWidth',1.0, ...
    'TickDir','out', ...
    'TickLength',[0.015 0.015], ...
    'XMinorTick','on', ...
    'YMinorTick','on', ...
    'Layer','top');

axh.GridAlpha      = 0.18;
axh.MinorGridAlpha = 0.10;
axh.GridColor      = [0.2 0.2 0.2];
axh.MinorGridColor = [0.5 0.5 0.5];

end

%% ======== 为每一帧建立"保持到下一个盒子"的索引 ========
function boxIdxOfFrame = buildBoxHoldIndex(x, y, th, x_box, y_box, th_box)

n  = numel(x);
nb = numel(x_box);

anchor = zeros(nb,1);

for i = 1:nb
    dpos = (x - x_box(i)).^2 + (y - y_box(i)).^2;
    dth = wrapToPiLocal(th - th_box(i));
    score = dpos + 0.05*(dth.^2);
    [~, anchor(i)] = min(score);
end

anchor = cummax(anchor);
anchor(anchor < 1) = 1;
anchor(anchor > n) = n;

for i = 2:nb
    if anchor(i) < anchor(i-1)
        anchor(i) = anchor(i-1);
    end
end

boxIdxOfFrame = ones(n,1);

for i = 1:nb-1
    k1 = anchor(i);
    k2 = anchor(i+1) - 1;
    if k2 < k1
        k2 = k1;
    end
    boxIdxOfFrame(k1:k2) = i;
end

boxIdxOfFrame(anchor(nb):n) = nb;
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
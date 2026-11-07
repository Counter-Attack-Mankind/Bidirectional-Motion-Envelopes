function PlotEFBoxesAndTrueSweptArea()
% PlotEFBoxesAndTrueSweptArea
% ------------------------------------------------------------
% 作用：
%   1) 用绿色边框画出 Nfe 个原始采样点对应的具身盒子
%   2) 对真实车辆做稠密插值，形成连续扫掠面积
%   3) 蓝色表示前进扫掠面积，红色表示后退扫掠面积
%
% 依赖：
%   params.ef.x, y, theta
%   params.ef.up/down/left/right  (或 a/b/c/d)
%   params.ef.v / params.ef.s      用于判定前进后退
%   params.ef.enriched_x/y/theta   若存在则优先用于真实扫掠
%   ddd2()                         你现有工程里已经有
%
% 说明：
%   - 具身盒子：严格按原始 Nfe 点画，不插值
%   - 真实扫掠：按 enriched 位姿（若存在）或原始位姿形成连续区域
% ------------------------------------------------------------

global params

assert(isfield(params,'ef'), 'params.ef 不存在');
assert(isfield(params,'vehicle'), 'params.vehicle 不存在');
assert(isfield(params,'environment'), 'params.environment 不存在');

%% ========== 读取原始 Nfe 点（用于具身盒子） ==========
x_box  = params.ef.x(:);
y_box  = params.ef.y(:);
th_box = params.ef.theta(:);

up_box    = pickField(params.ef,'up','a',zeros(size(x_box)));    up_box    = up_box(:);
left_box  = pickField(params.ef,'left','b',zeros(size(x_box)));  left_box  = left_box(:);
down_box  = pickField(params.ef,'down','c',zeros(size(x_box)));  down_box  = down_box(:);
right_box = pickField(params.ef,'right','d',zeros(size(x_box))); right_box = right_box(:);

nbox = min([numel(x_box), numel(y_box), numel(th_box), ...
            numel(up_box), numel(left_box), numel(down_box), numel(right_box)]);

x_box     = x_box(1:nbox);
y_box     = y_box(1:nbox);
th_box    = th_box(1:nbox);
up_box    = up_box(1:nbox);
left_box  = left_box(1:nbox);
down_box  = down_box(1:nbox);
right_box = right_box(1:nbox);

%% ========== 读取稠密位姿（用于真实车辆扫掠） ==========
use_enriched = isfield(params.ef,'enriched_x') && ~isempty(params.ef.enriched_x) && ...
               isfield(params.ef,'enriched_y') && ~isempty(params.ef.enriched_y) && ...
               isfield(params.ef,'enriched_theta') && ~isempty(params.ef.enriched_theta);

if use_enriched
    x  = params.ef.enriched_x(:);
    y  = params.ef.enriched_y(:);
    th = params.ef.enriched_theta(:);
else
    x  = params.ef.x(:);
    y  = params.ef.y(:);
    th = params.ef.theta(:);
end

n = min([numel(x), numel(y), numel(th)]);
x  = x(1:n);
y  = y(1:n);
th = th(1:n);

if n < 2
    error('轨迹点过少，无法构造扫掠区域');
end

%% ========== 方向信息 ==========
% 原始方向（用于给原始盒子分类，如果你以后想分类上色）
if isfield(params.ef,'s') && ~isempty(params.ef.s)
    s_box = params.ef.s(:);
elseif isfield(params.ef,'v') && ~isempty(params.ef.v) && isfield(params.ef,'dt') && ~isempty(params.ef.dt)
    vv = params.ef.v(:);
    dtt = params.ef.dt(:);
    m = min(numel(vv), numel(dtt));
    s_box = vv(1:m) .* dtt(1:m);
else
    s_box = nan(nbox,1);
end
if numel(s_box) < nbox
    s_box(end+1:nbox) = s_box(end);
end

% 稠密方向（用于真实扫掠分蓝/红）
if use_enriched && isfield(params.ef,'enriched_v') && ~isempty(params.ef.enriched_v)
    v_dense = params.ef.enriched_v(:);
elseif isfield(params.ef,'v') && ~isempty(params.ef.v)
    v0 = params.ef.v(:);
    if use_enriched
        boxIdxOfFrame = buildBoxHoldIndex(x, y, th, x_box, y_box, th_box);
        v_dense = nan(n,1);
        for k = 1:n
            idx = min(max(boxIdxOfFrame(k),1), numel(v0));
            v_dense(k) = v0(idx);
        end
    else
        v_dense = v0(:);
    end
else
    % 没有 v，则用相邻位移与朝向粗略判定
    v_dense = estimateDirectionFromPose(x, y, th);
end

if numel(v_dense) < n
    v_dense(end+1:n) = v_dense(end);
else
    v_dense = v_dense(1:n);
end

%% ========== 车辆参数 ==========
lf  = params.vehicle.lf;
lw  = params.vehicle.lw;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% ========== 建图 ==========
fig = figure('Name','EF boxes and true swept area','Color','w');
ax = axes(fig); hold(ax,'on'); grid(ax,'minor'); box(ax,'on'); axis(ax,'equal');

if isfield(params.environment,'xmin') && isfield(params.environment,'xmax') && ...
   isfield(params.environment,'ymin') && isfield(params.environment,'ymax')
    axis(ax, [params.environment.xmin, params.environment.xmax, ...
              params.environment.ymin, params.environment.ymax]);
end

xlabel(ax,'x / m');
ylabel(ax,'y / m');
% title(ax,'Embodiment Boxes and True Vehicle Swept Area');

%% ========== 画障碍物 ==========
if isfield(params.environment,'obs') && ~isempty(params.environment.obs)
    for ii = 1:length(params.environment.obs)
        fill(ax, params.environment.obs(ii).x, params.environment.obs(ii).y, ...
            [0.80 0.80 0.80], ...
            'EdgeColor', [0.45 0.45 0.45], ...
            'LineWidth', 0.8, ...
            'HandleVisibility','off');
    end
end

%% ========== 画参考轨迹 ==========
plot(ax, x, y, 'k--', 'LineWidth', 1.0, 'HandleVisibility','off');

%% ========== 构造真实车辆前进/后退扫掠 ==========
swept_forward = polyshape();
swept_reverse = polyshape();

eps_dir = 1e-10;

for k = 1:n
    [XB, YB] = body_rect_corners(x(k), y(k), th(k), lf, lw, lr, hlb);
    bodyPoly = polyshape(XB, YB);

    if v_dense(k) > eps_dir
        swept_forward = union(swept_forward, bodyPoly);
    elseif v_dense(k) < -eps_dir
        swept_reverse = union(swept_reverse, bodyPoly);
    else
        % 速度接近 0 时，可按前一帧延续；这里简单跳过
    end
end

%% ========== 先画真实扫掠面积 ==========
% 蓝：前进
hForward = plot(swept_forward, ...
    'FaceColor', [0.15 0.40 0.95], ...
    'FaceAlpha', 0.30, ...
    'EdgeColor', 'none');

% 红：后退
hReverse = plot(swept_reverse, ...
    'FaceColor', [0.90 0.15 0.15], ...
    'FaceAlpha', 0.30, ...
    'EdgeColor', 'none');

%% ========== 再画 Nfe 个具身盒子（按前进/后退着色） ==========
boxColorF = [52, 101, 164] / 255;   % 前进：深蓝
boxColorR = [180, 68, 68] / 255;    % 后退：深红
for i = 1:nbox
    [XE, YE] = ddd2_poly( ...
        x_box(i), y_box(i), th_box(i), ...
        up_box(i), left_box(i), down_box(i), right_box(i));

    % ===== 根据方向给具身盒子上色 =====
    if i <= numel(s_box)
        if s_box(i) > 1e-10
            edgeCol = boxColorF;   % 前进蓝
        elseif s_box(i) < -1e-10
            edgeCol = boxColorR;   % 后退红
        else
            edgeCol = [0 0 0];     % 停车/换向点可选黑色
        end
    else
        edgeCol = [0 0 0];
    end

    patch(ax, XE, YE, [1 1 1], ...
        'FaceColor', 'none', ...
        'EdgeColor', edgeCol, ...
        'LineWidth', 1.5, ...
        'HandleVisibility','off');
end

%% ========== 可选：画原始采样点 ==========
hSamples = plot(ax, x_box, y_box, 'ko', ...
    'MarkerSize', 3, ...
    'MarkerFaceColor', 'k');

%% ========== 图例 ==========
legBoxF = plot(ax, nan, nan, '-', 'Color', boxColorF, 'LineWidth', 1.5);
legBoxR = plot(ax, nan, nan, '-', 'Color', boxColorR, 'LineWidth', 1.5);
legF  = patch(nan, nan, [0.15 0.40 0.95], 'FaceAlpha', 0.30, 'EdgeColor', 'none');
legR  = patch(nan, nan, [0.90 0.15 0.15], 'FaceAlpha', 0.30, 'EdgeColor', 'none');

% legend(ax, [legBoxF, legBoxR, legF, legR, hSamples], ...
%     {'EF box (forward)', 'EF box (reverse)', ...
%      'True swept area (forward)', 'True swept area (reverse)', ...
%      'Nfe samples'}, ...
%     'Location', 'eastoutside');

fprintf('\n================ Plot Finished ================\n');
fprintf('原始具身盒子数 Nfe        : %d\n', nbox);
fprintf('真实扫掠稠密位姿点数      : %d\n', n);
fprintf('蓝色 = 前进真实扫掠区\n');
fprintf('红色 = 后退真实扫掠区\n');
fprintf('================================================\n\n');

drawnow;
SaveTaskFigure(gcf, 'EFboxs');

end


%% =========================================================
%                    辅助函数
%% =========================================================

function val = pickField(S, newName, oldName, defaultVal)
if isfield(S,newName) && ~isempty(S.(newName))
    val = S.(newName);
elseif isfield(S,oldName) && ~isempty(S.(oldName))
    val = S.(oldName);
else
    val = defaultVal;
end
end

function [X,Y] = ddd2_poly(x,y,theta,a,b,c,d)
[AX,AY,BX,BY,CX,CY,DX,DY] = ddd2(x,y,theta,a,b,c,d);
X = [AX BX CX DX AX];
Y = [AY BY CY DY AY];
end

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

function boxIdxOfFrame = buildBoxHoldIndex(x, y, th, x_box, y_box, th_box)
n  = numel(x);
nb = numel(x_box);

anchor = zeros(nb,1);

for i = 1:nb
    dpos = (x - x_box(i)).^2 + (y - y_box(i)).^2;
    dth  = wrapToPiLocal(th - th_box(i));
    score = dpos + 0.05*(dth.^2);
    [~, anchor(i)] = min(score);
end

anchor = cummax(anchor);
anchor(anchor < 1) = 1;
anchor(anchor > n) = n;

boxIdxOfFrame = ones(n,1);
for i = 1:nb-1
    k1 = anchor(i);
    k2 = anchor(i+1)-1;
    if k2 < k1
        k2 = k1;
    end
    boxIdxOfFrame(k1:k2) = i;
end
boxIdxOfFrame(anchor(nb):n) = nb;
boxIdxOfFrame(1:anchor(1)) = 1;
end

function a = wrapToPiLocal(a)
a = mod(a + pi, 2*pi) - pi;
end

function v_est = estimateDirectionFromPose(x, y, th)
n = numel(x);
v_est = zeros(n,1);

for k = 1:n-1
    dx = x(k+1) - x(k);
    dy = y(k+1) - y(k);
    tdir = [cos(th(k)); sin(th(k))];
    ds = [dx; dy];
    proj = dot(ds, tdir);

    if proj > 1e-10
        v_est(k) = 1;
    elseif proj < -1e-10
        v_est(k) = -1;
    else
        v_est(k) = 0;
    end
end

if n >= 2
    v_est(n) = v_est(n-1);
end
end
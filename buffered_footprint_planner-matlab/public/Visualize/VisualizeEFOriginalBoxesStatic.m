function VisualizeEFOriginalBoxesStatic()
% VisualizeEFOriginalBoxesStatic
% --------------------------------------------
% 使用 AMPL 原始优化采样点，静态画出每个采样点的具身盒子
% 不插值，不动画
%
% 前提：
%   已运行 LoadEFOptimumAndRefine()
%   并已将结果存入 params.ef
% --------------------------------------------

global params

%% ======== 读取原始优化结果 ========
x     = params.ef.x(:);
y     = params.ef.y(:);
theta = params.ef.theta(:);

up    = params.ef.up(:);
down  = params.ef.down(:);
left  = params.ef.left(:);
right = params.ef.right(:);

N = length(x);

%% ======== 若 up/down/left/right 是段变量(N-1)，则与NLP保持一致，只画1..N-1 ========
if length(up) == N-1
    Nplot = N-1;
else
    Nplot = N;
end

%% ======== 车辆几何参数 ========
LF  = params.vehicle.lf;   % 若你工程里前悬字段叫 LF，可改成 params.vehicle.LF
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% ======== 创建 Figure ========
fig = figure(randi(10000));
set(fig,'OuterPosition',get(0,'ScreenSize'));

ax = axes(fig);
hold(ax,'on'); grid(ax,'minor'); box(ax,'on'); axis(ax,'equal');

axis(ax,[params.environment.xmin, params.environment.xmax, ...
         params.environment.ymin, params.environment.ymax]);

set(ax,'FontWeight','bold','FontSize',18,'FontName','Arial Narrow');
xlabel(ax,'x / m');
ylabel(ax,'y / m');
title(ax,'Original EF Boxes (Static Check)');

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

%% ======== 轨迹与采样点 ========
plot(ax, x, y, 'k-', 'LineWidth', 1.2);
plot(ax, x, y, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');

%% ======== 逐点画具身盒子（严格按 NLP.mod 角点公式） ========
for i = 1:Nplot

    c = cos(theta(i));
    s = sin(theta(i));

    % ===== 按你的NLP.mod公式 =====
    AX = x(i) + (LF + up(i))   * c - (hlb + left(i))  * s;
    BX = x(i) + (LF + up(i))   * c + (hlb + right(i)) * s;
    CX = x(i) - (lr + down(i)) * c + (hlb + right(i)) * s;
    DX = x(i) - (lr + down(i)) * c - (hlb + left(i))  * s;

    AY = y(i) + (LF + up(i))   * s + (hlb + left(i))  * c;
    BY = y(i) + (LF + up(i))   * s - (hlb + right(i)) * c;
    CY = y(i) - (lr + down(i)) * s - (hlb + right(i)) * c;
    DY = y(i) - (lr + down(i)) * s + (hlb + left(i))  * c;

    PX = [AX; BX; CX; DX; AX];
    PY = [AY; BY; CY; DY; AY];

    patch(ax, PX, PY, [0.3 0.6 1], ...
        'FaceAlpha', 0.12, ...
        'EdgeColor', [0 0.35 0.95], ...
        'LineWidth', 0.8, ...
        'HandleVisibility','off');

    % 如果你想显示编号，可以打开这一行
    % text(x(i), y(i), sprintf('%d',i), 'FontSize',8, 'Color',[0.2 0.2 0.2]);
end

%% ======== 起终点 ========
plot(ax, x(1),   y(1),   'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(ax, x(end), y(end), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');

end
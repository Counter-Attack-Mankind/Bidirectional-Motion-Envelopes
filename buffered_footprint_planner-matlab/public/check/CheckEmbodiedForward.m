function CheckEmbodiedForward()

clc; close all;

%% ========= 路径 =========
path = 'D:\desktop\embodyfootprint-hybrid\results\';

%% ========= 读取 AMPL 结果 =========
s      = load([path 's.txt']);
k      = load([path 'k.txt']);
up     = load([path 'up.txt']);
down   = load([path 'down.txt']);
left   = load([path 'left.txt']);
right  = load([path 'right.txt']);

s      = s(:);
k      = k(:);
up     = up(:);
down   = down(:);
left   = left(:);
right  = right(:);

N = length(s);

assert(length(k)     == N, 'k.txt 长度不一致');
assert(length(up)    == N, 'up.txt 长度不一致');
assert(length(down)  == N, 'down.txt 长度不一致');
assert(length(left)  == N, 'left.txt 长度不一致');
assert(length(right) == N, 'right.txt 长度不一致');

%% ========= 车辆参数（务必与 PV 完全一致） =========
global params
LF  = params.vehicle.LF;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% ========= 阈值 =========
tol = 1e-10;   % 用于区分前进/倒车/近零段

idxF = find(s >  tol);   % 前进段
idxR = find(s < -tol);   % 倒车段
idxZ = find(abs(s) <= tol); % 近零段（通常换向点附近）

fprintf('总采样点数: %d\n', N);
fprintf('前进段采样点数: %d\n', length(idxF));
fprintf('倒车段采样点数: %d\n', length(idxR));
fprintf('近零段采样点数: %d\n', length(idxZ));

%% ========= 理论值初始化 =========
up_th    = zeros(N,1);
down_th  = zeros(N,1);
left_th  = zeros(N,1);
right_th = zeros(N,1);

%% ========= 前进段理论值 =========
for ii = 1:length(idxF)
    id = idxF(ii);

    si = s(id);      % si > 0
    ki = k(id);

    up_th(id)    = si + hlb * abs(ki) * si;
    down_th(id)  = 0;
    left_th(id)  = max(-lr * ki * si, (LF + 0.5 * si) * ki * si);
    right_th(id) = max( lr * ki * si, -(LF + 0.5 * si) * ki * si);
end

%% ========= 倒车段理论值 =========
for ii = 1:length(idxR)
    id = idxR(ii);

    si = s(id);      % si < 0
    ki = k(id);

    up_th(id)    = 0;
    down_th(id)  = -si + hlb * abs(ki) * (-si);
    left_th(id)  = max(-lr * ki * si + 0.5 * ki * si^2, LF * ki * si);
    right_th(id) = max(-LF * ki * si, lr * ki * si - 0.5 * ki * si^2);
end

%% ========= 近零段理论值 =========
% 对 |s| 很小的点，统一按 0 处理，避免换向点附近数值歧义
up_th(idxZ)    = 0;
down_th(idxZ)  = 0;
left_th(idxZ)  = 0;
right_th(idxZ) = 0;

%% ========= 误差 =========
err_up    = abs(up_th    - up);
err_down  = abs(down_th  - down);
err_left  = abs(left_th  - left);
err_right = abs(right_th - right);

fprintf('\n========== 总体误差统计 ==========\n');
fprintf('up    max error: %.6e\n', max(err_up));
fprintf('down  max error: %.6e\n', max(err_down));
fprintf('left  max error: %.6e\n', max(err_left));
fprintf('right max error: %.6e\n', max(err_right));

[~, iup]    = max(err_up);
[~, idown]  = max(err_down);
[~, ileft]  = max(err_left);
[~, iright] = max(err_right);

fprintf('\n最大误差点:\n');
fprintf('up    idx=%d\n', iup);
fprintf('down  idx=%d\n', idown);
fprintf('left  idx=%d\n', ileft);
fprintf('right idx=%d\n', iright);

%% ========= 分段误差统计 =========
if ~isempty(idxF)
    fprintf('\n========== 前进段误差统计 ==========\n');
    fprintf('up    max error: %.6e\n', max(err_up(idxF)));
    fprintf('down  max error: %.6e\n', max(err_down(idxF)));
    fprintf('left  max error: %.6e\n', max(err_left(idxF)));
    fprintf('right max error: %.6e\n', max(err_right(idxF)));
end

if ~isempty(idxR)
    fprintf('\n========== 倒车段误差统计 ==========\n');
    fprintf('up    max error: %.6e\n', max(err_up(idxR)));
    fprintf('down  max error: %.6e\n', max(err_down(idxR)));
    fprintf('left  max error: %.6e\n', max(err_left(idxR)));
    fprintf('right max error: %.6e\n', max(err_right(idxR)));
end

%% ========= 打印前进段最大误差点详情 =========
if ~isempty(idxF)
    [~, locF] = max(err_up(idxF) + err_down(idxF) + err_left(idxF) + err_right(idxF));
    idF = idxF(locF);

    fprintf('\n===== 前进段代表性检查点 idx=%d =====\n', idF);
    fprintf('s      = %.12f\n', s(idF));
    fprintf('k      = %.12f\n', k(idF));
    fprintf('AMPL:  up=%.12f, down=%.12f, left=%.12f, right=%.12f\n', ...
        up(idF), down(idF), left(idF), right(idF));
    fprintf('THRY:  up=%.12f, down=%.12f, left=%.12f, right=%.12f\n', ...
        up_th(idF), down_th(idF), left_th(idF), right_th(idF));
end

%% ========= 打印倒车段最大误差点详情 =========
if ~isempty(idxR)
    [~, locR] = max(err_up(idxR) + err_down(idxR) + err_left(idxR) + err_right(idxR));
    idR = idxR(locR);

    fprintf('\n===== 倒车段代表性检查点 idx=%d =====\n', idR);
    fprintf('s      = %.12f\n', s(idR));
    fprintf('k      = %.12f\n', k(idR));
    fprintf('AMPL:  up=%.12f, down=%.12f, left=%.12f, right=%.12f\n', ...
        up(idR), down(idR), left(idR), right(idR));
    fprintf('THRY:  up=%.12f, down=%.12f, left=%.12f, right=%.12f\n', ...
        up_th(idR), down_th(idR), left_th(idR), right_th(idR));
end

%% ========= 误差图 =========
figure('Name','Embodied Scale Error Check','Color','w');

subplot(2,2,1);
plot(err_up, 'LineWidth', 1.5); hold on;
plot(idxF, err_up(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, err_up(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('up error');
xlabel('index'); ylabel('abs error');
legend('error','forward','reverse');

subplot(2,2,2);
plot(err_down, 'LineWidth', 1.5); hold on;
plot(idxF, err_down(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, err_down(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('down error');
xlabel('index'); ylabel('abs error');
legend('error','forward','reverse');

subplot(2,2,3);
plot(err_left, 'LineWidth', 1.5); hold on;
plot(idxF, err_left(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, err_left(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('left error');
xlabel('index'); ylabel('abs error');
legend('error','forward','reverse');

subplot(2,2,4);
plot(err_right, 'LineWidth', 1.5); hold on;
plot(idxF, err_right(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, err_right(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('right error');
xlabel('index'); ylabel('abs error');
legend('error','forward','reverse');

%% ========= 理论值与AMPL值对比图 =========
figure('Name','Embodied Scale Theory vs AMPL','Color','w');

subplot(2,2,1);
plot(up, 'r-', 'LineWidth', 1.2); hold on;
plot(up_th, 'b-', 'LineWidth', 1.2);
grid on;
title('up: AMPL vs Theory');
legend('AMPL','Theory');

subplot(2,2,2);
plot(down, 'r-', 'LineWidth', 1.2); hold on;
plot(down_th, 'b-', 'LineWidth', 1.2);
grid on;
title('down: AMPL vs Theory');
legend('AMPL','Theory');

subplot(2,2,3);
plot(left, 'r-', 'LineWidth', 1.2); hold on;
plot(left_th, 'b-', 'LineWidth', 1.2);
grid on;
title('left: AMPL vs Theory');
legend('AMPL','Theory');

subplot(2,2,4);
plot(right, 'r-', 'LineWidth', 1.2); hold on;
plot(right_th, 'b-', 'LineWidth', 1.2);
grid on;
title('right: AMPL vs Theory');
legend('AMPL','Theory');

%% ========= 差值曲线图（AMPL - Theory） =========
diff_up    = up    - up_th;
diff_down  = down  - down_th;
diff_left  = left  - left_th;
diff_right = right - right_th;

figure('Name','Embodied Scale Difference Curves','Color','w');

subplot(2,2,1);
plot(diff_up, 'k-', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.0);
plot(idxF, diff_up(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, diff_up(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('up difference: AMPL - Theory');
xlabel('index');
ylabel('\Delta up');
legend('\Delta up','zero line','forward','reverse');

subplot(2,2,2);
plot(diff_down, 'k-', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.0);
plot(idxF, diff_down(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, diff_down(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('down difference: AMPL - Theory');
xlabel('index');
ylabel('\Delta down');
legend('\Delta down','zero line','forward','reverse');

subplot(2,2,3);
plot(diff_left, 'k-', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.0);
plot(idxF, diff_left(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, diff_left(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('left difference: AMPL - Theory');
xlabel('index');
ylabel('\Delta left');
legend('\Delta left','zero line','forward','reverse');

subplot(2,2,4);
plot(diff_right, 'k-', 'LineWidth', 1.5); hold on;
yline(0, 'r--', 'LineWidth', 1.0);
plot(idxF, diff_right(idxF), 'bo', 'MarkerSize', 3);
plot(idxR, diff_right(idxR), 'ro', 'MarkerSize', 3);
grid on;
title('right difference: AMPL - Theory');
xlabel('index');
ylabel('\Delta right');
legend('\Delta right','zero line','forward','reverse');

end
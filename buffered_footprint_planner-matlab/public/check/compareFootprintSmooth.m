function compareFootprintSmooth()
% LEGACY DIAGNOSTIC: compares the retired LSE footprint with exact max.
%% =======================
% 车辆参数
global params
lf  = params.vehicle.LF;
lr  = params.vehicle.lr;
hlb = params.vehicle.hlb;

%% =======================
% 结果路径
result_dir = 'D:\desktop\具身足迹毕设\embodyfootprint-hybrid\results';

%% =======================
% 读取优化后的外扩尺度
left_opt  = load(fullfile(result_dir,'left.txt'));
right_opt = load(fullfile(result_dir,'right.txt'));
up_opt    = load(fullfile(result_dir,'up.txt'));
down_opt  = load(fullfile(result_dir,'down.txt'));

k  = load(fullfile(result_dir,'k.txt'));
s = load(fullfile(result_dir,'s.txt'));


% 保证维度一致
k  = k(:);
left_opt  = left_opt(:);
right_opt = right_opt(:);
up_opt    = up_opt(:);
down_opt  = down_opt(:);

% 若 k 比区间数多一个点，则截断到区间长度
N = length(s);
if length(k) > N
    k = k(1:N);
end

%% =======================
% 理论值计算（区分前进/倒车/近零段）
tol = 0.00001;   % 区分前进/倒车/近零段
idxF = find(s > tol);      % 前进段
idxR = find(s < -tol);     % 倒车段
idxZ = find(abs(s) <= tol);% 近零段

% 初始化理论值
left_theo  = zeros(N,1);
right_theo = zeros(N,1);
up_theo    = zeros(N,1);
down_theo  = zeros(N,1);

% 前进段
for i = 1:length(idxF)
    id = idxF(i);
    si = s(id);  % >0
    ki = k(id);
    up_theo(id)    = si + hlb * abs(ki) * si;
    down_theo(id)  = 0;
    left_theo(id)  = max(-lr*ki*si, (lf + 0.5*si)*ki*si);
    right_theo(id) = max( lr*ki*si, -(lf + 0.5*si)*ki*si);
end

% 倒车段
for i = 1:length(idxR)
    id = idxR(i);
    si = s(id);  % <0
    ki = k(id);
    up_theo(id)    = 0;
    down_theo(id)  = -si + hlb * abs(ki) * (-si);
    left_theo(id)  = max(-lr*ki*si + 0.5*ki*si^2, lf*ki*si);
    right_theo(id) = max(-lf*ki*si, lr*ki*si - 0.5*ki*si^2);
end

% 近零段统一为0
up_theo(idxZ)    = 0;
down_theo(idxZ)  = 0;
left_theo(idxZ)  = 0;
right_theo(idxZ) = 0;

%% =======================
% 差值计算：光滑 - 理论
diff_left  = left_opt  - left_theo;
diff_right = right_opt - right_theo;
diff_up    = up_opt    - up_theo;
diff_down  = down_opt  - down_theo;

% 绝对误差（用于统计）
error_left  = abs(diff_left);
error_right = abs(diff_right);
error_up    = abs(diff_up);
error_down  = abs(diff_down);

fprintf('平均绝对误差 left: %.6f, max: %.6f\n', mean(error_left), max(error_left));
fprintf('平均绝对误差 right: %.6f, max: %.6f\n', mean(error_right), max(error_right));
fprintf('平均绝对误差 up: %.6f, max: %.6f\n', mean(error_up), max(error_up));
fprintf('平均绝对误差 down: %.6f, max: %.6f\n', mean(error_down), max(error_down));

fprintf('平均带符号误差 left: %.6f\n', mean(diff_left));
fprintf('平均带符号误差 right: %.6f\n', mean(diff_right));
fprintf('平均带符号误差 up: %.6f\n', mean(diff_up));
fprintf('平均带符号误差 down: %.6f\n', mean(diff_down));

%% =======================
% 检查是否存在 smooth < theoretical 的情况
tol = 1e-5;   % 数值容差，避免浮点误差导致误判

idx_left_neg  = find(diff_left  < -tol);
idx_right_neg = find(diff_right < -tol);
idx_up_neg    = find(diff_up    < -tol);
idx_down_neg  = find(diff_down  < -tol);

fprintf('\n================ 负差值检查（smooth - theoretical < 0） ================\n');

% left
if isempty(idx_left_neg)
    fprintf('left: 未发现小于 0 的点，满足 smooth >= theoretical\n');
else
    fprintf('left: 发现 %d 个点小于 0\n', length(idx_left_neg));
    fprintf('最小差值 = %.12f\n', min(diff_left));
    fprintf('对应索引及数值如下：\n');
    for i = 1:length(idx_left_neg)
        id = idx_left_neg(i);
        fprintf('  idx = %d, diff_left = %.12f, left_opt = %.12f, left_theo = %.12f\n', ...
            id, diff_left(id), left_opt(id), left_theo(id));
    end
end

% right
if isempty(idx_right_neg)
    fprintf('right: 未发现小于 0 的点，满足 smooth >= theoretical\n');
else
    fprintf('right: 发现 %d 个点小于 0\n', length(idx_right_neg));
    fprintf('最小差值 = %.12f\n', min(diff_right));
    fprintf('对应索引及数值如下：\n');
    for i = 1:length(idx_right_neg)
        id = idx_right_neg(i);
        fprintf('  idx = %d, diff_right = %.12f, right_opt = %.12f, right_theo = %.12f\n', ...
            id, diff_right(id), right_opt(id), right_theo(id));
    end
end

% up
if isempty(idx_up_neg)
    fprintf('up: 未发现小于 0 的点，满足 smooth >= theoretical\n');
else
    fprintf('up: 发现 %d 个点小于 0\n', length(idx_up_neg));
    fprintf('最小差值 = %.12f\n', min(diff_up));
    fprintf('对应索引及数值如下：\n');
    for i = 1:length(idx_up_neg)
        id = idx_up_neg(i);
        fprintf('  idx = %d, diff_up = %.12f, up_opt = %.12f, up_theo = %.12f\n', ...
            id, diff_up(id), up_opt(id), up_theo(id));
    end
end

% down
if isempty(idx_down_neg)
    fprintf('down: 未发现小于 0 的点，满足 smooth >= theoretical\n');
else
    fprintf('down: 发现 %d 个点小于 0\n', length(idx_down_neg));
    fprintf('最小差值 = %.12f\n', min(diff_down));
    fprintf('对应索引及数值如下：\n');
    for i = 1:length(idx_down_neg)
        id = idx_down_neg(i);
        fprintf('  idx = %d, diff_down = %.12f, down_opt = %.12f, down_theo = %.12f\n', ...
            id, diff_down(id), down_opt(id), down_theo(id));
    end
end

%% =======================
% 可视化：直接画差值曲线（改 legend）
t = 1:N;
figure('Position',[100 100 1200 800]);

% 左
subplot(2,2,1); hold on; grid on;
h1 = plot(t, diff_left, 'b-', 'LineWidth', 2);
h2 = yline(0, 'r--', 'LineWidth', 1.5);
if ~isempty(idx_left_neg)
    h3 = plot(idx_left_neg, diff_left(idx_left_neg), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
    legend([h1 h2 h3], '\Delta = smooth - theoretical', 'y = 0', 'negative points', 'Location', 'best');
else
    legend([h1 h2], '\Delta = smooth - theoretical', 'y = 0', 'Location', 'best');
end
title('Left: smooth - theoretical');
xlabel(sprintf('Index (Max |Error| = %.4f)', max(error_left)));
ylabel('\Delta left');

% 右
subplot(2,2,2); hold on; grid on;
h1 = plot(t, diff_right, 'b-', 'LineWidth', 2);
h2 = yline(0, 'r--', 'LineWidth', 1.5);
if ~isempty(idx_right_neg)
    h3 = plot(idx_right_neg, diff_right(idx_right_neg), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
    legend([h1 h2 h3], '\Delta = smooth - theoretical', 'y = 0', 'negative points', 'Location', 'best');
else
    legend([h1 h2], '\Delta = smooth - theoretical', 'y = 0', 'Location', 'best');
end
title('Right: smooth - theoretical');
xlabel(sprintf('Index (Max |Error| = %.4f)', max(error_right)));
ylabel('\Delta right');

% 上
subplot(2,2,3); hold on; grid on;
h1 = plot(t, diff_up, 'b-', 'LineWidth', 2);
h2 = yline(0, 'r--', 'LineWidth', 1.5);
if ~isempty(idx_up_neg)
    h3 = plot(idx_up_neg, diff_up(idx_up_neg), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
    legend([h1 h2 h3], '\Delta = smooth - theoretical', 'y = 0', 'negative points', 'Location', 'best');
else
    legend([h1 h2], '\Delta = smooth - theoretical', 'y = 0', 'Location', 'best');
end
title('Up: smooth - theoretical');
xlabel(sprintf('Index (Max |Error| = %.4f)', max(error_up)));
ylabel('\Delta up');

% 下
subplot(2,2,4); hold on; grid on;
h1 = plot(t, diff_down, 'b-', 'LineWidth', 2);
h2 = yline(0, 'r--', 'LineWidth', 1.5);
if ~isempty(idx_down_neg)
    h3 = plot(idx_down_neg, diff_down(idx_down_neg), 'ro', 'MarkerSize', 6, 'LineWidth', 1.5);
    legend([h1 h2 h3], '\Delta = smooth - theoretical', 'y = 0', 'negative points', 'Location', 'best');
else
    legend([h1 h2], '\Delta = smooth - theoretical', 'y = 0', 'Location', 'best');
end
title('Down: smooth - theoretical');
xlabel(sprintf('Index (Max |Error| = %.4f)', max(error_down)));
ylabel('\Delta down');

%% =======================
% 理论值与光滑值叠加 + 差值填充（顺序：左、右、上、下）
t = (1:N)';

figure('Position',[100 100 1200 800]);

fill_color = [0.96 0.80 0.72];   % 学术肉色（偏浅、低饱和）
alpha_val  = 1;               % 稍微降低一点透明度更柔和

% ===== left =====
subplot(2,2,1); hold on; grid on;

upper = max(left_opt, left_theo);
lower = min(left_opt, left_theo);

fill([t; flipud(t)], [upper; flipud(lower)], ...
    fill_color, 'EdgeColor','none', 'FaceAlpha',alpha_val);

plot(t, left_opt,  'r-',  'LineWidth', 1.8);
plot(t, left_theo, 'b--', 'LineWidth', 1.8);

title('Left: Smooth vs Theoretical');
xlabel('Index');
ylabel('Left scale');
legend('smooth - theory', 'Smooth', 'Theoretical', 'Location', 'best');

% ===== right =====
subplot(2,2,2); hold on; grid on;

upper = max(right_opt, right_theo);
lower = min(right_opt, right_theo);

fill([t; flipud(t)], [upper; flipud(lower)], ...
    fill_color, 'EdgeColor','none', 'FaceAlpha',alpha_val);

plot(t, right_opt,  'r-',  'LineWidth', 1.8);
plot(t, right_theo, 'b--', 'LineWidth', 1.8);

title('Right: Smooth vs Theoretical');
xlabel('Index');
ylabel('Right scale');
legend('smooth - theory', 'Smooth', 'Theoretical', 'Location', 'best');

% ===== up =====
subplot(2,2,3); hold on; grid on;

upper = max(up_opt, up_theo);
lower = min(up_opt, up_theo);

fill([t; flipud(t)], [upper; flipud(lower)], ...
    fill_color, 'EdgeColor','none', 'FaceAlpha',alpha_val);

plot(t, up_opt,  'r-',  'LineWidth', 1.8);
plot(t, up_theo, 'b--', 'LineWidth', 1.8);

title('Up: Smooth vs Theoretical');
xlabel('Index');
ylabel('Up scale');
legend('smooth - theory', 'Smooth', 'Theoretical', 'Location', 'best');

% ===== down =====
subplot(2,2,4); hold on; grid on;

upper = max(down_opt, down_theo);
lower = min(down_opt, down_theo);

fill([t; flipud(t)], [upper; flipud(lower)], ...
    fill_color, 'EdgeColor','none', 'FaceAlpha',alpha_val);

plot(t, down_opt,  'r-',  'LineWidth', 1.8);
plot(t, down_theo, 'b--', 'LineWidth', 1.8);

title('Down: Smooth vs Theoretical');
xlabel('Index');
ylabel('Down scale');
legend('smooth - theory', 'Smooth', 'Theoretical', 'Location', 'best');

sgtitle('Footprint Scale with Deviation Band (Smooth vs Theoretical)', ...
    'FontSize', 14, 'FontWeight', 'bold');
end

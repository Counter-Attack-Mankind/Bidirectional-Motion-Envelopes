function PlotAMPLStaticTraj()

%% ====== 文件路径 ======
base_path = 'D:\desktop\具身足迹毕设\embodyfootprint-hybrid\results';   

file_x      = fullfile(base_path, 'x.txt');
file_y      = fullfile(base_path, 'y.txt');
file_s      = fullfile(base_path, 's.txt');
file_v      = fullfile(base_path, 'v.txt');
file_dt     = fullfile(base_path, 'dt.txt');
file_a      = fullfile(base_path, 'a.txt');
file_theta  = fullfile(base_path, 'theta.txt');
file_kappa  = fullfile(base_path, 'k.txt');
file_w      = fullfile(base_path, 'w.txt');
file_eup    = fullfile(base_path, 'up.txt');
file_eright = fullfile(base_path, 'right.txt');
file_eleft  = fullfile(base_path, 'left.txt');
file_edown  = fullfile(base_path, 'down.txt');


%% ====== 读取数据 ======
x      = load(file_x);      x = x(:);
y      = load(file_y);      y = y(:);
s      = load(file_s);      s = s(:);
v      = load(file_v);      v = v(:);
dt     = load(file_dt);     dt = dt(:);
a      = load(file_a);      a = a(:);
theta  = load(file_theta);  theta = theta(:);
kappa  = load(file_kappa);  kappa = kappa(:);
w      = load(file_w);      w = w(:);
eup    = load(file_eup);    eup = eup(:);
eright = load(file_eright); eright = eright(:);
eleft  = load(file_eleft);  eleft = eleft(:);
edown  = load(file_edown);  edown = edown(:);

%% ====== 主状态长度统一（不含 dt）======
N = min([ ...
    length(x), length(y), length(s),length(v), length(a), length(theta), ...
    length(kappa), length(w), length(eup), length(eright), ...
    length(eleft), length(edown)]);

x      = x(1:N);
y      = y(1:N);
v      = v(1:N);
s      = s(1:N);
a      = a(1:N);
theta  = theta(1:N);
kappa  = kappa(1:N);
w      = w(1:N);
eup    = eup(1:N);
eright = eright(1:N);
eleft  = eleft(1:N);
edown  = edown(1:N);

idx_all = (1:N).';

%% ====== dt 单独索引 ======
Ndt = length(dt);
idx_dt = (1:Ndt).';

%% ====== 检测换向点 ======
thr = 1e-3;

dir_sign = zeros(size(s));
dir_sign(s >  thr) = 1;
dir_sign(s < -thr) = -1;

% 对 0 点做方向传播
for i = 2:N
    if dir_sign(i) == 0
        dir_sign(i) = dir_sign(i-1);
    end
end
for i = N-1:-1:1
    if dir_sign(i) == 0
        dir_sign(i) = dir_sign(i+1);
    end
end

change_idx = find(dir_sign(2:end) ~= dir_sign(1:end-1)) + 1;

%% ====== 图 1：轨迹图 ======
figure('Color','w','Name','Trajectory');
hold on;

plot(x, y, 'b-', 'LineWidth', 1.5);
plot(x, y, 'ro', 'MarkerSize', 4, 'LineWidth', 1.0);
plot(x(1), y(1), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
plot(x(end), y(end), 'ms', 'MarkerSize', 10, 'LineWidth', 2);

if ~isempty(change_idx)
    plot(x(change_idx), y(change_idx), 'kp', 'MarkerSize', 12, 'LineWidth', 2);
end

axis equal;
grid on;
xlabel('x');
ylabel('y');
title('Static Trajectory with Sample Points and Gear Change Points');

legend_entries = {'Trajectory', 'Sample Points', 'Start', 'End'};
if ~isempty(change_idx)
    legend_entries{end+1} = 'Change Points';
end
legend(legend_entries, 'Location', 'best');

%% ====== 轨迹图中标注采样点序号（可选） ======
show_index = true;
step = 5;

if show_index
    for i = 1:step:N
        text(x(i), y(i), sprintf('%d', i), ...
            'FontSize', 8, ...
            'Color', 'k', ...
            'VerticalAlignment', 'bottom', ...
            'HorizontalAlignment', 'left');
    end
end

%% ====== 图 2：显示 v(dt用s代表当前行驶方向量)、dt、s、kappa 与外扩尺度 ======
figure('Color','w','Name','Key States and Embodiment Margins');

subplot(4,2,1);
plot(idx_all, v, 'b-', 'LineWidth', 1.2); hold on; grid on;
plot_change_lines(change_idx, v);
xlabel('Sample Index'); ylabel('v');
title('v');

subplot(4,2,2);
plot(idx_dt, dt, 'b-', 'LineWidth', 1.2); hold on; grid on;
xlabel('Segment Index'); ylabel('dt');
title('dt');

subplot(4,2,3);
plot(idx_all, s, 'b-', 'LineWidth', 1.2); hold on; grid on;
plot_change_lines(change_idx, s);
xlabel('Sample Index'); ylabel('s');
title('s');

subplot(4,2,4);
plot(idx_all, kappa, 'b-', 'LineWidth', 1.2); hold on; grid on;
plot_change_lines(change_idx, kappa);
xlabel('Sample Index'); ylabel('\kappa');
title('\kappa');

subplot(4,2,5);
plot(idx_all, eup, 'b-', 'LineWidth', 1.2); hold on; grid on;
plot_change_lines(change_idx, eup);
xlabel('Sample Index'); ylabel('e_{up}');
title('e_{up}');

subplot(4,2,6);
plot(idx_all, eright, 'r-', 'LineWidth', 1.2); hold on; grid on;
plot(idx_all, eleft,  'g-', 'LineWidth', 1.2);
plot(idx_all, edown,  'm-', 'LineWidth', 1.2);
plot_change_lines(change_idx, [eright,eleft,edown]);
xlabel('Sample Index'); ylabel('e');
title('e_{right}, e_{left}, e_{down}');
legend({'e_{right}','e_{left}','e_{down}'}, 'Location','best');

subplot(4,2,[7 8]);
hold on; grid on;
plot(idx_all, eup,    'b-', 'LineWidth', 1.5);
plot(idx_all, eright, 'r-', 'LineWidth', 1.5);
plot(idx_all, eleft,  'g-', 'LineWidth', 1.5);
plot(idx_all, edown,  'm-', 'LineWidth', 1.5);
e_min = min([eup, eright, eleft, edown], [], 2);
plot(idx_all, e_min, 'k--', 'LineWidth', 2.0);
plot_change_lines(change_idx, [eup,eright,eleft,edown]);
xlabel('Sample Index'); ylabel('Embodiment Margins');
title('Integrated Embodiment Margins');
legend({'e_{up}','e_{right}','e_{left}','e_{down}','e_{min}'}, 'Location','best');

sgtitle('v, dt, s, \kappa and Embodiment Margins');

%% ====== 图 3：表格化查看所有采样点状态 ======
T = table(idx_all, x, y, v, a, theta, kappa, w, eup, eright, eleft, edown, ...
    'VariableNames', {'idx','x','y','v','a','theta','kappa','w','eup','eright','eleft','edown'});

disp('================ 所有采样点状态表 ================');
disp(T);

%% ====== 打印换向点信息 ======
fprintf('总采样点数: %d\n', N);
fprintf('检测到换向次数: %d\n', numel(change_idx));

for k = 1:numel(change_idx)
    idx = change_idx(k);
    fprintf(['换向点 #%d:\n' ...
             '  idx   = %d\n' ...
             '  x     = %.6f\n' ...
             '  y     = %.6f\n' ...
             '  v     = %.6f\n' ...
             '  a     = %.6f\n' ...
             '  theta = %.6f\n' ...
             '  kappa = %.6f\n' ...
             '  w     = %.6f\n' ...
             '  eup   = %.6f\n' ...
             '  eright= %.6f\n' ...
             '  eleft = %.6f\n' ...
             '  edown = %.6f\n\n'], ...
             k, idx, x(idx), y(idx), v(idx), a(idx), theta(idx), ...
             kappa(idx), w(idx), eup(idx), eright(idx), eleft(idx), edown(idx));
end

%% ====== 打印换向点信息 ======
fprintf('总采样点数: %d\n', N);
fprintf('检测到换向次数: %d\n', numel(change_idx));

for k = 1:numel(change_idx)
    idx = change_idx(k);
    fprintf(['换向点 #%d:\n' ...
             '  idx   = %d\n' ...
             '  x     = %.6f\n' ...
             '  y     = %.6f\n' ...
             '  s     = %.6f\n' ...
             '  a     = %.6f\n' ...
             '  theta = %.6f\n' ...
             '  kappa = %.6f\n' ...
             '  w     = %.6f\n' ...
             '  eup   = %.6f\n' ...
             '  eright= %.6f\n' ...
             '  eleft = %.6f\n' ...
             '  edown = %.6f\n\n'], ...
             k, idx, x(idx), y(idx), s(idx), a(idx), theta(idx), ...
             kappa(idx), w(idx), eup(idx), eright(idx), eleft(idx), edown(idx));
end

end

%% ====== 辅助函数：在状态图中标出换向点 ======
function plot_change_lines(change_idx, data)
if isempty(change_idx)
    return;
end

yl = ylim;
for i = 1:numel(change_idx)
    xline(change_idx(i), '--k', 'LineWidth', 1.0);
end

if isvector(data)
    valid_idx = change_idx(change_idx >= 1 & change_idx <= length(data));
    plot(valid_idx, data(valid_idx), 'kp', 'MarkerSize', 8, 'LineWidth', 1.5);
end
ylim(yl);
end
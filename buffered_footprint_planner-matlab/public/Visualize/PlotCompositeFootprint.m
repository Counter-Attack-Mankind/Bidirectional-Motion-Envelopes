function PlotCompositeFootprint()

%% ====== 文件路径 ======
base_path = 'D:\desktop\具身足迹毕设\embodyfootprint-hybrid\results';   

file_x      = fullfile(base_path, 'x.txt');
file_y      = fullfile(base_path, 'y.txt');
file_eup    = fullfile(base_path, 'up.txt');
file_eright = fullfile(base_path, 'right.txt');
file_eleft  = fullfile(base_path, 'left.txt');
file_edown  = fullfile(base_path, 'down.txt');

%% ====== 读取数据 ======
x      = load(file_x);      x = x(:);
y      = load(file_y);      y = y(:);
eup    = load(file_eup);    eup = eup(:);
eright = load(file_eright); eright = eright(:);
eleft  = load(file_eleft);  eleft = eleft(:);
edown  = load(file_edown);  edown = edown(:);

N = min([length(x), length(y), length(eup), length(eright), length(eleft), length(edown)]);
x      = x(1:N);
y      = y(1:N);
eup    = eup(1:N);
eright = eright(1:N);
eleft  = eleft(1:N);
edown  = edown(1:N);

idx_all = (1:N).';

%% ====== 检测换向点 ======
thr = 1e-3;

% 使用原始弧长 s 文件数据
file_s = fullfile(base_path, 's.txt');
s = load(file_s); s = s(:);
s = s(1:N);

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

change_idx = find(dir_sign(2:end) ~= dir_sign(1:end-1));

%% ====== 图7：综合外扩尺度 ======
figure('Color','w','Name','综合外扩尺度');
hold on; grid on;

plot(idx_all, eup,    'b-', 'LineWidth', 1.5);
plot(idx_all, eright, 'r-', 'LineWidth', 1.5);
plot(idx_all, eleft,  'g-', 'LineWidth', 1.5);
plot(idx_all, edown,  'm-', 'LineWidth', 1.5);

% 最小外扩尺度
e_min = min([eup, eright, eleft, edown], [], 2);
plot(idx_all, e_min, 'k--', 'LineWidth', 2.0);

% 绘制换向点
if ~isempty(change_idx)
    % xline(change_idx, '--k', 'LineWidth', 1.0);
    % plot(change_idx, eup(change_idx),    'kp', 'MarkerSize', 8, 'LineWidth', 1.5);
    % plot(change_idx, eright(change_idx), 'kp', 'MarkerSize', 8, 'LineWidth', 1.5);
    % plot(change_idx, eleft(change_idx),  'kp', 'MarkerSize', 8, 'LineWidth', 1.5);
    % plot(change_idx, edown(change_idx),  'kp', 'MarkerSize', 8, 'LineWidth', 1.5);
end

xlabel('配置点序列');
ylabel('外扩尺度 / m');
title('图7 综合外扩尺度');
legend({'e_{up}','e_{right}','e_{left}','e_{down}','e_{min}'}, 'Location', 'best');

end
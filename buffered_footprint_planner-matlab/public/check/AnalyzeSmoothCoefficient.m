function AnalyzeSmoothCoefficientFull(mat_file)
% LEGACY DIAGNOSTIC: retained only for analysis of the retired LSE model.
% AnalyzeSmoothCoefficientFull - 综合分析 log-sum-exp 光滑系数 alpha
%
% 输入:
%   mat_file - 初始解保存的 .mat 文件路径, 默认 'written_initial_guess_data.mat'
%
% 功能:
%   1. 输出 splus/sminus/k 的统计量
%   2. 分析 left/right/up/down 的组合量级
%   3. 给出综合推荐 alpha 范围
%   4. 可视化不同 alpha 下四方向平滑误差曲线

%% =======================
% 默认文件
if nargin < 1
    mat_file = 'written_initial_guess_data.mat';
end
if ~isfile(mat_file)
    error('文件不存在: %s', mat_file);
end

data = load(mat_file);

splus  = data.data.splus(:);
sminus = data.data.sminus(:);
k      = data.data.kappa(:);

% 车辆参数（从初始解 meta 里读取）
params = data.data.meta;
LF  = params.LF;   % 前轮到前轴
lr  = params.lr;   % 后轮到后轴
hlb = params.hlb;  % 半车宽

%% =======================
% 四方向组合量级计算
left_terms  = [-lr*k.*splus, (LF+0.5*splus).*k.*splus, -LF*k.*sminus, (lr+0.5*sminus).*k.*sminus];
right_terms = [lr*k.*splus, -(LF+0.5*splus).*k.*splus, LF*k.*sminus, -(lr+0.5*sminus).*k.*sminus];
up_terms    = splus + hlb*abs(k).*splus;
down_terms  = sminus + hlb*abs(k).*sminus;

% 取每行/元素绝对值最大
left_max_val  = max(abs(left_terms), [], 2);
right_max_val = max(abs(right_terms), [], 2);
up_max_val    = max(abs(up_terms));
down_max_val  = max(abs(down_terms));

% 综合最大值
overall_max = max([left_max_val; right_max_val; up_max_val; down_max_val]);
fprintf('组合量级最大值 (综合四方向) = %.6f\n', overall_max);

%% =======================
% 推荐 alpha 范围
alpha_min = 5 / overall_max;
alpha_max = 50 / overall_max;
fprintf('推荐 alpha 范围: %.2f ~ %.2f\n', alpha_min, alpha_max);

%% =======================
% 推荐 alpha 范围（固定区间 5~40，步长 5）
alphas = 50:10:100;  % 5,10,15,20,25,30,35,40
N = length(splus);

figure('Position',[100 100 1200 600]); clf;
colors = lines(length(alphas));

for i = 1:length(alphas)
    alpha_val = alphas(i);
    
    % 左平滑
    left_smooth  = (1/alpha_val) * log( exp(alpha_val.*(-lr*k.*splus)) + exp(alpha_val.*((LF+0.5*splus).*k.*splus)) ) ...
             + (1/alpha_val) * log( exp(alpha_val.*(-LF*k.*sminus)) + exp(alpha_val.*((lr+0.5*sminus).*k.*sminus)) );

    % 右平滑
    right_smooth = (1/alpha_val) * log( exp(alpha_val.*(lr*k.*splus)) + exp(alpha_val.*(-(LF+0.5*splus).*k.*splus)) ) ...
             + (1/alpha_val) * log( exp(alpha_val.*(LF*k.*sminus)) + exp(alpha_val.*(-(lr+0.5*sminus).*k.*sminus)) );
             
    up_smooth    = up_terms;   % 线性
    down_smooth  = down_terms; % 线性
    
    % 理论 max
    left_max  = max(-lr*k.*splus, (LF+0.5*splus).*k.*splus) + max(-LF*k.*sminus, (lr+0.5*sminus).*k.*sminus);
    right_max = max(lr*k.*splus, -(LF+0.5*splus).*k.*splus) + max(LF*k.*sminus, -(lr+0.5*sminus).*k.*sminus);
    up_max    = up_terms;
    down_max  = down_terms;
    
    % 误差
    error_left  = abs(left_max - left_smooth);
    error_right = abs(right_max - right_smooth);
    error_up    = abs(up_max - up_smooth);
    error_down  = abs(down_max - down_smooth);
    
    % 绘图
    subplot(2,2,1); hold on; plot(1:N, error_left, 'Color', colors(i,:), 'LineWidth', 1.5); title('Left error'); xlabel('i'); ylabel('误差'); grid on;
    subplot(2,2,2); hold on; plot(1:N, error_right, 'Color', colors(i,:), 'LineWidth', 1.5); title('Right error'); xlabel('i'); ylabel('误差'); grid on;
    subplot(2,2,3); hold on; plot(1:N, error_up, 'Color', colors(i,:), 'LineWidth', 1.5); title('Up error'); xlabel('i'); ylabel('误差'); grid on;
    subplot(2,2,4); hold on; plot(1:N, error_down, 'Color', colors(i,:), 'LineWidth', 1.5); title('Down error'); xlabel('i'); ylabel('误差'); grid on;
end

legend_strings = arrayfun(@(a) sprintf('\\alpha=%.2f',a), alphas, 'UniformOutput', false);
for j = 1:4
    subplot(2,2,j); legend(legend_strings,'Location','best'); 
end
sgtitle('不同 alpha 下四方向 log-sum-exp 平滑误差');

end

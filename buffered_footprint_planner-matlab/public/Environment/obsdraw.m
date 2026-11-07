clc; clear; close all;

% 参数
N = 50000;                 % 随机点数量
range = 1.5;               % 采样范围 [-range, range]
p_list = [1, 2, 4, 10, 50, 100];  % 不同 p 值

% 随机采样
x = (rand(N,1)*2 - 1) * range;
y = (rand(N,1)*2 - 1) * range;

figure;

for k = 1:length(p_list)
    
    p = p_list(k);
    
    % p-norm 判定：|x|^p + |y|^p <= 1
    inside = (abs(x).^p + abs(y).^p) <= 1;
    
    subplot(2,3,k);
    hold on;
    

    
    % 内部点（红色）
    plot(x(inside), y(inside), '.k', 'MarkerSize', 2);
    
    axis equal;
    xlim([-range range]);
    ylim([-range range]);
    
    title(['p = ', num2str(p)]);
    
end